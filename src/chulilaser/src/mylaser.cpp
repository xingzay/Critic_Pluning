#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>



//  处理雷达切割角度完成！！

class RadarSubscriber : public rclcpp::Node
{
public:
    RadarSubscriber() : Node("radar_subscriber")
    {
        // 创建雷达数据的订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser_scan", 10, std::bind(&RadarSubscriber::radarCallback, this, std::placeholders::_1));
        // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "depth_scan", 10, std::bind(&RadarSubscriber::radarCallback2, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan_c", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RadarSubscriber::fuseAndPublish, this));
    }

private:
    void radarCallback(const sensor_msgs::msg::LaserScan::SharedPtr radar_data)
    {
        radar_data1 = *radar_data;
        radar_data1.intensities.clear();
        // publisher_->publish(radar_data1);
    }

    // void radarCallback2(const sensor_msgs::msg::LaserScan::SharedPtr radar_data)
    // {
    //     // 处理雷达数据的回调函数
    //     // 这里可以编写您需要的雷达数据处理逻辑
    //     radar_data2 = *radar_data;
    //     // radar_data2.header.frame_id= "base_link";
    //     // radar_data2.angle_min = radar_data->angle_min/2;
    //     // radar_data2.angle_max = -radar_data->angle_min/2;
    //     // for (size_t i = 0; i < radar_data2.ranges.size(); ++i)
    //     // {
    //     //     if (std::isnan(radar_data2.ranges[i]))
    //     //     {
    //     //         radar_data2.ranges[i] = std::numeric_limits<float>::infinity();
    //     //     }
            
    //     // }

    // }

    void fuseAndPublish()
    {   
        radar_dataL = radar_data1;

        publisher_->publish(radar_dataL);

    }
    sensor_msgs::msg::LaserScan radar_data1;
    sensor_msgs::msg::LaserScan radar_data2;
    sensor_msgs::msg::LaserScan radar_dataL;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}