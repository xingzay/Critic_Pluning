
#include "dwb_critics/path_align.hpp"
#include <vector>
#include <string>
#include "dwb_critics/alignment_util.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace dwb_critics
{

void PathAlignCritic::onInit()
{
  PathDistCritic::onInit();
  stop_on_failure_ = false;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  forward_point_distance_ = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".forward_point_distance", 0.325); //默认值
}

bool PathAlignCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  double dx = pose.x - goal.x;
  double dy = pose.y - goal.y;
  double sq_dist = dx * dx + dy * dy;
  if (sq_dist > forward_point_distance_ * forward_point_distance_) 
  {
    zero_scale_ = false;
  } 
  else 
  {
    // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
    zero_scale_ = true;
    return true;
  }
  //如果没到前瞻位置，处理距离评估 - 这个距离是机器人当前位置到局部路径上的一个点(全局路径的一部分)，以便确定这个点是否在本地代价地图中
  //处理距离评估
  //RCLCPP_INFO(rclcpp::get_logger("logger"), "Path_Align准备被使用");
  return PathDistCritic::prepare(pose, vel, goal, global_plan);
}

double PathAlignCritic:: getScale() const
{
  if (zero_scale_) {
    return 0.0;  
  } else {
    return costmap_->getResolution() * 0.5 * scale_;
  }
}

double PathAlignCritic::scorePose(const geometry_msgs::msg::Pose2D & pose)
{
  // scorePose - 获取 前瞻路径点 通过在代价地图上的单元格的下标索引来获取该单元格的代价值，返回的也是代价值
  //getForwardPose 获取前瞻位置的全局坐标点(x,y,theta) - 局部路径终点前距离forward_point_distance_的路径点 
  return PathDistCritic::scorePose(getForwardPose(pose, forward_point_distance_));
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathAlignCritic, dwb_core::TrajectoryCritic)
