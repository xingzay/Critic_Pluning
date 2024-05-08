#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

class check_turtlebot3_node : public rclcpp::Node
{
private:
    
  // 心跳监测
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publish_heartbeat();

public:
    check_turtlebot3_node();
    ~check_turtlebot3_node();
};
