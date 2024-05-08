#include "turtlebot3_test/check_turtlebot3_node.hpp"

check_turtlebot3_node::check_turtlebot3_node():Node("check_turtlebot3_node"){
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Empty>("/heart_beat", 10);
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&check_turtlebot3_node::publish_heartbeat, this)
    );
}
check_turtlebot3_node::~check_turtlebot3_node(){

}

void check_turtlebot3_node::publish_heartbeat(){
    auto msg = std::make_shared<std_msgs::msg::Empty>();
    heartbeat_pub_->publish(*msg);  // 发布心跳信号
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<check_turtlebot3_node>();
    rclcpp::spin(node);  
    rclcpp::shutdown();  
    return 0;
}
