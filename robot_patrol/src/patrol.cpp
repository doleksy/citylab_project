#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

using std::placeholders::_1;

constexpr const double rad2deg = 180/3.14159265358979323846;

class Patrol : public rclcpp::Node
{
public:
    Patrol()
        : Node{ "Patrol" }
        , laserSub_{ this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
            std::bind(&Patrol::laserScanCb, this, _1)) }
    {
        RCLCPP_INFO(this->get_logger(), "Constructing Patrol");
    }

private:
    static constexpr int neg_pi_2 { 165 };
    static constexpr int pos_pi_2 { 495 };

    void laserScanCb(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "LaserScan callback triggered: size: %lu", msg->ranges.size());

        const auto rangeMax{ msg->range_max };
        const auto &rng = msg->ranges;
        const auto maxDistanceIter =
            std::max_element(
                std::next(rng.begin(), neg_pi_2),
                std::next(rng.begin(), pos_pi_2),
                [rangeMax](float a, float b) { return (a > rangeMax ? 0 : a) < (b > rangeMax ? 0 : b); } );
        const auto maxDistanceIdx = std::distance(rng.begin(), maxDistanceIter);
        const auto zeroIdx{ UINT32_C(msg->ranges.size() / 2) };
        distance_ = (maxDistanceIdx - zeroIdx) * msg->angle_increment;

        RCLCPP_INFO(get_logger(), "  maxDist: %f at idx: %lu(%lu) and angle: %f (%f degrees)",
            *maxDistanceIter, maxDistanceIdx, maxDistanceIdx - zeroIdx, distance_, distance_*rad2deg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
    float distance_{};

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
