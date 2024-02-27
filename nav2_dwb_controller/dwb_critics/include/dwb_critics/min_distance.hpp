#ifndef DWB_CRITICS__MIN_DISTANCE_HPP_
#define DWB_CRITICS__MIN_DISTANCE_HPP_
#include <vector>
#include <string>
#include <chrono>

#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "costmap_queue/costmap_queue.hpp"

using namespace std::chrono_literals; 
namespace dwb_critics
{
class Min_DistanceCritic : public dwb_core::TrajectoryCritic{
public:
    Min_DistanceCritic();
    bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
    double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override ;
private:
struct ThetaResult {
    double theta;
    double theta2;
};
nav2_costmap_2d::Costmap2D * costmap_;
nav_2d_msgs::msg::Path2D global_plan_;
bool zero_scale_;
rclcpp::Logger logger_{rclcpp::get_logger("Min_Distance")};
rclcpp::Clock::SharedPtr clock_;

std::pair<size_t, size_t> FindCloseGoalPlannerPoints(
    const dwb_msgs::msg::Trajectory2D &traj,
    const nav_2d_msgs::msg::Path2D &transformed_plan);
size_t FindClosestPoint(const geometry_msgs::msg::Pose2D &point, const nav_2d_msgs::msg::Path2D &path);

ThetaResult CountTheta(size_t num1, size_t num2, nav_2d_msgs::msg::Path2D &transformed_plan);

double InsertPointandComputeDistance(
    ThetaResult &theta, size_t num1, size_t num2,
    const dwb_msgs::msg::Trajectory2D &traj,
    nav_2d_msgs::msg::Path2D &transformed_plan);
};
}
#endif