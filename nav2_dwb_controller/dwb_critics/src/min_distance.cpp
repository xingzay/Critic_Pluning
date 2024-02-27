#include "dwb_critics/min_distance.hpp"

namespace dwb_critics{

Min_DistanceCritic::Min_DistanceCritic() : zero_scale_(false) {}

bool Min_DistanceCritic::prepare(
  const geometry_msgs::msg::Pose2D & ,
  const nav_2d_msgs::msg::Twist2D & ,
  const geometry_msgs::msg::Pose2D & , 
  const nav_2d_msgs::msg::Path2D & global_plan)
{
 // RCLCPP_INFO(this->logger_, "Min_Distance 准备被使用！");
  global_plan_ = global_plan;
  return true;
}

double Min_DistanceCritic::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj){
  //RCLCPP_INFO(this->logger_, "Min_Distance 开始被使用！");

  nav_2d_msgs::msg::Path2D transformed_plan;
  transformed_plan = global_plan_;
  transformed_plan.poses = global_plan_.poses;

  // 在函数里面要实现找到最小距离且要找到最小距离处的那个下标i
  std::pair<size_t, size_t> closest_points = FindCloseGoalPlannerPoints(traj, transformed_plan);
  size_t num1_pose_index = closest_points.first;
  size_t num2_pose_index = closest_points.second;
  ThetaResult theta = CountTheta(num1_pose_index,num2_pose_index,transformed_plan);
  //RCLCPP_INFO(this->logger_, "找到角度了");
  double min_dis = InsertPointandComputeDistance(theta, num1_pose_index, num2_pose_index, traj, transformed_plan);
  //RCLCPP_INFO(this->logger_, "此时的距离为：%f cm",min_dis);
  //dwb_msgs::msg::CriticScore critic_score = convertFloatToCriticScore(min_dis);
  return min_dis;
}

//如果保证 num1_pose_index < num2_pose_index ？
//通过交换？
std::pair<size_t, size_t> Min_DistanceCritic::FindCloseGoalPlannerPoints(const dwb_msgs::msg::Trajectory2D& traj, 
  const nav_2d_msgs::msg::Path2D& transformed_plan) 
{
  size_t num1_pose_index = FindClosestPoint(traj.poses.front(), transformed_plan);
  size_t num2_pose_index = FindClosestPoint(traj.poses.back(), transformed_plan);
  //RCLCPP_INFO(this->logger_, "找到num1和num2了！");
  if (num1_pose_index == num2_pose_index)
  {
    if (num1_pose_index > 0 && num1_pose_index < transformed_plan.poses.size() - 1) {
      double distance_to_start = std::abs(transformed_plan.poses.front().x - transformed_plan.poses[num1_pose_index].x);
      double distance_to_end = std::abs(transformed_plan.poses[num1_pose_index].x - transformed_plan.poses.back().x);
      if (distance_to_start < distance_to_end){
        num2_pose_index = num1_pose_index + 1;
      }else{
        num1_pose_index = num1_pose_index - 1;
      }
    } 
    else {
      if (num1_pose_index == 0) {
        num2_pose_index = num1_pose_index + 1;
      }
      else if (num1_pose_index == transformed_plan.poses.size() - 1)
      {
        num1_pose_index = num1_pose_index - 1;
      }
    }
  }
  else {
    if (num1_pose_index > num2_pose_index) {
      std::swap(num1_pose_index, num2_pose_index);
    }
  }
  return std::make_pair(num1_pose_index, num2_pose_index);
}

size_t Min_DistanceCritic::FindClosestPoint(const geometry_msgs::msg::Pose2D &point, 
  const nav_2d_msgs::msg::Path2D &path){
  double min_distance = std::numeric_limits<double>::infinity();
  size_t closest_index = std::numeric_limits<size_t>::max();

  for (size_t i = 0; i < path.poses.size(); ++i){
    double dis_x = std::abs(point.x - path.poses[i].x);
    double dis_y = std::abs(point.y - path.poses[i].y);
    double distance = std::hypot(dis_x, dis_y);
    if (distance < min_distance){
      min_distance = distance;
      closest_index = i;
    }
  }
  if(min_distance == std::numeric_limits<double>::infinity() ) {
    RCLCPP_ERROR(this->logger_, "未找到最近的路径点 -1");
    return std::numeric_limits<size_t>::max();
  }
  return closest_index;
}

dwb_critics::Min_DistanceCritic::ThetaResult Min_DistanceCritic::CountTheta(
  size_t num1, size_t num2, nav_2d_msgs::msg::Path2D &transformed_plan)
{
  double x = transformed_plan.poses[num2].x - transformed_plan.poses[num1].x;
  double y = transformed_plan.poses[num2].y - transformed_plan.poses[num1].y;
  ThetaResult result;
  if (y >= 0) {
    result.theta = std::atan2(y, x);
    result.theta2 = std::numeric_limits<double>::quiet_NaN();  // 未使用的值设为 NaN
  } else {
    double y2 = -y;
    result.theta = std::numeric_limits<double>::quiet_NaN();  // 未使用的值设为 NaN
    result.theta2 = std::atan2(y2, x);
  }
  return result;
}

double Min_DistanceCritic::InsertPointandComputeDistance(
  ThetaResult& thetaresult, size_t num1, size_t num2,
  const dwb_msgs::msg::Trajectory2D &traj,
  nav_2d_msgs::msg::Path2D &transformed_plan)
{
  size_t i = 0;
  size_t j = num1;
  double Min_Distance = std::numeric_limits<double>::infinity();
  //循环计算距离，返回最小值
  while (i < traj.poses.size() && j < num2 - 1 && j+1 < transformed_plan.poses.size()){
    transformed_plan.poses[j + 1].x = traj.poses[i].x;
   if (!std::isnan(thetaresult.theta)){
      double dis_x = traj.poses[i].x - transformed_plan.poses[num1].x;
      transformed_plan.poses[j + 1].y = dis_x * tan(thetaresult.theta) + transformed_plan.poses[num1].y;
    }
    else if(!std::isnan(thetaresult.theta2)){
      double dis_x = std::abs(traj.poses[i].x - transformed_plan.poses[num2].x);
      transformed_plan.poses[j + 1].y = dis_x * tan(thetaresult.theta2) + transformed_plan.poses[num2].y;
    }
    double distance = std::abs(traj.poses[i].y - transformed_plan.poses[j + 1].y);
    distance = distance * 100;
    if (distance < Min_Distance)
    {
      Min_Distance = distance;
    }
    ++i;
    ++j;
  }
  return Min_Distance;
}
} // namespace dwb_critics
PLUGINLIB_EXPORT_CLASS(dwb_critics::Min_DistanceCritic, dwb_core::TrajectoryCritic)