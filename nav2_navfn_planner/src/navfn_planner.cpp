// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "nav2_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_navfn_planner
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type NavfnPlanner",
    name_.c_str());
}

void
NavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type NavfnPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_astar", use_astar_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  declare_parameter_if_not_declared(node, name + ".use_Bezier", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_Bezier", use_Bezier_);

  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
}

void NavfnPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NavfnPlanner::dynamicParametersCallback, this, _1));
}

void NavfnPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
  dyn_params_handler_.reset();
}

void NavfnPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type NavfnPlanner",
    name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  // Update planner based on the new costmap size
  //根据新的成本计算图大小更新规划器
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  //如果起始点与目标点相同的情况
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
      return path;
    }
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner

    /*  如果起始方向和目标方向不同，则将唯一路径姿态设置为目标方向，
       除非 use_final_approach_orientation=true 
        在这种情况下，我们需要将其设置为起始方向，以避免本地规划器的移动
    */
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) 
    {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
    return path;
  }
  // 规划路径入口函数
  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    RCLCPP_WARN(
      logger_, "%s: failed to create plan with "
      "tolerance %.2f.", name_.c_str(), tolerance_);
  }


#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  std::cout << "It took " << time_span.count() * 1000 << std::endl;
#endif

  return path;
}

bool NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

// 规划路径入口函数
// 搜索路径的时候是从目标点往起始点搜索，存储路径的时候是从起始点开始到目标点
bool NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  //判断起点是否超出了全局地图
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "Cannot create a plan: the robot's start position is off the global"
      " costmap. Planning will always fail, are you sure"
      " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap(清除代价地图中的起始单元格--设置为free) because we know it can't be an obstacle
  clearRobotCell(mx, my);
  // 获取互斥锁 -- 上锁
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  lock.unlock();

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;
  //判断终点是否超出了全局地图
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "The goal sent to the planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  // 选择 Dijkstra / A*
  if (use_astar_) 
  {
    planner_->calcNavFnAstar();
  }
  else
  {
    planner_->calcNavFnDijkstra(true); 
  }

  
  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p = goal;
  //获取当前点在map地图上的代价值
  double potential = getPointPotential(p.position);

  //判断当前点是否能到达目标点
  if (potential < POT_HIGH) {  // 能直接到达
    best_pose = p;
    found_legal = true;
  } else {
    //不能直接到达 -- 抛弃原先当前点，重新寻找目标点附近容差范围内可以直接到达的点，取代之前的当前点
    double best_sdist = std::numeric_limits<double>::max();
    //以p点为中心点，向下和向左为正方形的区域，遍布这个区域内的点(当前点),容差为tolerance
    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }
  
  //路径规划的生成
  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      if(use_Bezier_){
        // 在这里加一个贝塞尔曲线优化整条路径，优化完后再进行路径末尾的优化
        nav_msgs::msg::Path optimized_path;
        optimizePathWithBezier(plan, optimized_path);
        // 将优化后的路径赋值给 plan
        plan = optimized_path;
      }
       // 路径末尾优化
       smoothApproachToGoal(best_pose, plan);
       // 如果，use_final_approach_orientation_ = true，则处理最后一个位姿的方向（插值处理），避免旋转
       // 如何处理：找到最后一个位姿和前一个位姿的角度偏差theta，沿 Z 轴旋转theta度
       if (use_final_approach_orientation_)
       {
         size_t plan_size = plan.poses.size();
         if (plan_size == 1)
         {
           plan.poses.back().pose.orientation = start.orientation;
         }
         else if (plan_size > 1)
         {
           double dx, dy, theta;
           auto last_pose = plan.poses.back().pose.position;
           auto approach_pose = plan.poses[plan_size - 2].pose.position; // 前一个位姿的方向
           // Deal with the case of NavFn producing a path with two equal last poses
           // 前后位置相同，处理最后一个位姿和前两个的位姿
           if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
               std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
           {
             approach_pose = plan.poses[plan_size - 3].pose.position;
           }
           dx = last_pose.x - approach_pose.x;
           dy = last_pose.y - approach_pose.y;
           theta = atan2(dy, dx);
           plan.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
         }
      }
    } 
    else 
    {
      RCLCPP_ERROR(
        logger_,
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}
double bernstein(int n, int i, double t) {
    return std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1)) * std::pow(t, i) * std::pow(1 - t, n - i);
}
// 贝塞尔曲线优化全局路径
void NavfnPlanner::optimizePathWithBezier(const nav_msgs::msg::Path & original_path, nav_msgs::msg::Path & optimized_path){
  int num_samples = 100;
    // 清空优化后的路径
    optimized_path.poses.clear();
    optimized_path.header = original_path.header;
    // 获取原始路径的控制点
    std::vector<geometry_msgs::msg::PoseStamped> control_points = original_path.poses;
    // 获取控制点的数量
    int n = control_points.size() - 1;
    // 为贝塞尔曲线生成采样点
    for (int sample = 0; sample <= num_samples; ++sample) {
        double t = static_cast<double>(sample) / num_samples;
        geometry_msgs::msg::PoseStamped new_pose;
        double x = 0.0;
        double y = 0.0;
        // 计算贝塞尔曲线的位置
        for (int i = 0; i <= n; ++i) {
          double coef = bernstein(n, i, t);
          x += coef * control_points[i].pose.position.x;
          y += coef * control_points[i].pose.position.y;
        }
        new_pose.pose.position.x = x;
        new_pose.pose.position.y = y;
        optimized_path.poses.push_back(new_pose);
    }
}

// 处理路径末尾优化
//如果最后一个路径点与倒数第二个路径点的距离大于倒数第二个路径点与目标点的距离，那么将最后一个路径点替换为目标点，以实现更平滑的路径
void NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}
//获取路径规划
bool NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  //目标点超出代价地图，返回false
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  //根据代价地图大小获取最大迭代次数 max_cycles
  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);
  //进行路径规划，得到路径长度path_len
  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }
  //获取规划器的最后一次规划代价，并输出调试信息。
  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(
    logger_,
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  //提取规划器的路径点坐标
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  //将路径点坐标转化为全局坐标
  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  //world_point -- goal
  unsigned int mx, my;
  //将世界坐标系坐标转化为map代价地图上的网格坐标
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max(); //转化失败，则潜力值无穷大
  }
  //栅格地图上的网格坐标计算--12=1*10+2(跟这个一样)
  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

// bool
// NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
// {
//   return validPointPotential(world_point, tolerance_);
// }

// bool
// NavfnPlanner::validPointPotential(
//   const geometry_msgs::msg::Point & world_point, double tolerance)
// {
//   const double resolution = costmap_->getResolution();

//   geometry_msgs::msg::Point p = world_point;
//   double potential = getPointPotential(p);
//   if (potential < POT_HIGH) {
//     // world_point is reachable by itself
//     return true;
//   } else {
//     // world_point, is not reachable. Trying to find any
//     // reachable point within its tolerance region
//     p.y = world_point.y - tolerance;
//     while (p.y <= world_point.y + tolerance) {
//       p.x = world_point.x - tolerance;
//       while (p.x <= world_point.x + tolerance) {
//         potential = getPointPotential(p);
//         if (potential < POT_HIGH) {
//           return true;
//         }
//         p.x += resolution;
//       }
//       p.y += resolution;
//     }
//   }

//   return false;
// }

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}
//图像坐标系map到世界坐标系world的x，y坐标的转换  mx,my--单元格的坐标   wx，wy--全局地图的坐标
void NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

rcl_interfaces::msg::SetParametersResult
NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_navfn_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
