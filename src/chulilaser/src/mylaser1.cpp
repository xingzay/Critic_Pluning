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
            "scan", 10, std::bind(&RadarSubscriber::radarCallback, this, std::placeholders::_1));
        // subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "scan1", 10, std::bind(&RadarSubscriber::radarCallback2, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scanL", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RadarSubscriber::fuseAndPublish, this));
    }

private:
    void radarCallback(const sensor_msgs::msg::LaserScan::SharedPtr radar_data)
    {
        // 处理雷达数据的回调函数
        // 这里可以编写您需要的雷达数据处理逻辑
        radar_data1 = *radar_data;

        radar_data1.header.frame_id = "base_link";

        radar_data1.angle_max = (*radar_data).angle_max/4;
        radar_data1.angle_min = (*radar_data).angle_min/4;
        
        // int x = -(3/4*radar_data1.angle_min/radar_data1.angle_increment)-1;
        int x = -((3 * (*radar_data).angle_min) / (4 * (*radar_data).angle_increment)) - 1;

        int y = M_PI/2/(*radar_data).angle_increment;


        for (int i = x, j = 0; i < y+x && j < y; i++,j++)
        {
            radar_data1.ranges[j] = (*radar_data).ranges[i];
            radar_data1.intensities[j] = (*radar_data).intensities[i];
        }

            // 获取要删除的位置之后的迭代器
        // auto it = radar_dataL.ranges.begin() + y + 1;
        // radar_dataL.ranges.erase(it, radar_dataL.ranges.end());
        radar_data1.ranges.resize(y-1);
        radar_data1.intensities.resize(y-1);
        // auto it2 = radar_dataL.intensities.begin() + y + 1;
        // radar_dataL.intensities.erase(it2, radar_dataL.intensities.end());

    }

    // void radarCallback2(const sensor_msgs::msg::LaserScan::SharedPtr radar_data)
    // {
    //     // 处理雷达数据的回调函数
    //     // 这里可以编写您需要的雷达数据处理逻辑
    //     radar_data2 = *radar_data;

    //     for (size_t i = 0; i < radar_data2.ranges.size(); ++i)
    //     {
    //         if (std::isnan(radar_data2.ranges[i]))
    //         {
    //             radar_data2.ranges[i] = std::numeric_limits<float>::infinity();
    //         }
            
    //     }

    // }

    void fuseAndPublish()
    {   
        radar_dataL = radar_data1;

        // radar_dataL.angle_max = radar_data1.angle_max/4;
        // radar_dataL.angle_min = radar_data1.angle_min/4;

        // // int x = -(3/4*radar_data1.angle_min/radar_data1.angle_increment)-1;
        // int x = -((3 * radar_data1.angle_min) / (4 * radar_data1.angle_increment)) - 1;

        // int y = M_PI/2/radar_data1.angle_increment;


        // for (int i = x, j = 0; i < y+x && j < y; i++,j++)
        // {
        //     radar_dataL.ranges[j] = radar_data1.ranges[i];
        //     radar_dataL.intensities[j] = radar_data1.intensities[i];
        // }

        //     // 获取要删除的位置之后的迭代器
        // // auto it = radar_dataL.ranges.begin() + y + 1;
        // // radar_dataL.ranges.erase(it, radar_dataL.ranges.end());
        // radar_dataL.ranges.resize(y-1);
        // radar_dataL.intensities.resize(y-1);
        // // auto it2 = radar_dataL.intensities.begin() + y + 1;
        // // radar_dataL.intensities.erase(it2, radar_dataL.intensities.end());

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