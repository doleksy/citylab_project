#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>


class Patrol : public rclcpp::Node
{
public:
    Patrol()
        : Node("Patrol")
    {
        RCLCPP_INFO(this->get_logger(), "Constructing Patrol");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

    RCLCPP_INFO(patrol_node->get_logger(), "Running Main");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(patrol_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}