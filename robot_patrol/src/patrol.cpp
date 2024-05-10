#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


namespace
{
    constexpr double rad2deg { 180/3.14159265358979323846 };
    constexpr int neg_pi_2_idx { 165 };
    constexpr int pos_pi_2_idx { 495 };
}

class Patrol : public rclcpp::Node
{
public:
    Patrol()
        : Node{ "Patrol" }
        , laserScanCb_callbackGroup{ this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
        , cmdVelPub_{ this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10) }
        , timerCb_callbackGroup{ this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
        , timer_{ this->create_wall_timer(1000ms, std::bind(&Patrol::timerCb_, this), timerCb_callbackGroup) }
    {
        RCLCPP_INFO(this->get_logger(), "Constructing Patrol");

        rclcpp::SubscriptionOptions laserSubOptions;
        laserSubOptions.callback_group = laserScanCb_callbackGroup;
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
            std::bind(&Patrol::laserScanCb, this, _1), laserSubOptions);
    }

private:
    void laserScanCb(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "LaserScan callback triggered: size: %lu", msg->ranges.size());

        const auto rangeMax{ msg->range_max };
        const auto &rng = msg->ranges;
        const auto maxDistanceIter =
            std::max_element(
                std::next(rng.begin(), neg_pi_2_idx),
                std::next(rng.begin(), pos_pi_2_idx),
                [rangeMax](float a, float b) { return (a > rangeMax ? 0 : a) < (b > rangeMax ? 0 : b); } );
        const auto maxDistanceIdx = std::distance(rng.begin(), maxDistanceIter);
        const auto zeroIdx{ UINT32_C(msg->ranges.size() / 2) };
        distance_ = static_cast<int>(maxDistanceIdx - zeroIdx) * msg->angle_increment;

        RCLCPP_INFO(get_logger(), "  maxDist: %f at idx: %lu(%d) and angle: %f (%f degrees)",
            *maxDistanceIter, maxDistanceIdx, static_cast<int>(maxDistanceIdx - zeroIdx), distance_, distance_*rad2deg);
    }

    void timerCb_()
    {
        RCLCPP_INFO(get_logger(), "timer callback triggered");
    }

private:
    rclcpp::CallbackGroup::SharedPtr laserScanCb_callbackGroup;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
    
    rclcpp::CallbackGroup::SharedPtr timerCb_callbackGroup;
    rclcpp::TimerBase::SharedPtr timer_;

    float distance_{};

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto patrol_node = std::make_shared<Patrol>();

    RCLCPP_INFO(patrol_node->get_logger(), "Running Main");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(patrol_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
