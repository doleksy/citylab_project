#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"

#include "citylab_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

using LaserMsg = sensor_msgs::msg::LaserScan;
using ServiceType = citylab_interfaces::srv::GetDirection;
using TwistMsg = geometry_msgs::msg::Twist;


class Patrol : public rclcpp::Node
{
public:
    Patrol()
        : Node{ "PatrolWithService" }
        , laserScanCb_callbackGroup{ this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
        , cmdVelPub_{ this->create_publisher<TwistMsg>("cmd_vel", 10) }
        , client_{ create_client<ServiceType>("direction_service") }
        , timerCb_callbackGroup{ this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
        , timer_{ this->create_wall_timer(100ms, std::bind(&Patrol::timerCb_, this), timerCb_callbackGroup) }
    {
        RCLCPP_INFO(this->get_logger(), "Constructing PatrolWithService.");

        rclcpp::SubscriptionOptions laserSubOptions;
        laserSubOptions.callback_group = laserScanCb_callbackGroup;
        laserSub_ = this->create_subscription<LaserMsg>("scan", 10,
            std::bind(&Patrol::laserScanCb, this, _1), laserSubOptions);

        RCLCPP_INFO(this->get_logger(), "PatrolWithService Constructed");
    }

private:
    void laserScanCb(LaserMsg::SharedPtr msg)
    {
        RCLCPP_DEBUG(get_logger(), "LaserScan callback triggered");

        last_laser_ = *msg;
    }

    void timerCb_()
    {
        RCLCPP_DEBUG(get_logger(), "timer callback triggered");

        auto request = std::make_shared<ServiceType::Request>();
        request->laser_data = last_laser_;

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);
        const std::string direction{ result.get()->direction };
        RCLCPP_INFO(get_logger(), "direction: %s", direction.c_str());

        TwistMsg twistMsg;
        twistMsg.linear.x = 0.1f;

        if (direction == "forward")
        {
            twistMsg.angular.z = 0.0f;
        }
        else if (direction == "left")
        {
            twistMsg.angular.z = 0.5f;
        }
        else if (direction == "right")
        {
            twistMsg.angular.z = -0.5f;
        }
        else
        {
            twistMsg.linear.x = 0.0f;
        }

        cmdVelPub_->publish(twistMsg);
    }

private:
    rclcpp::CallbackGroup::SharedPtr laserScanCb_callbackGroup;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

    LaserMsg last_laser_{};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;

    rclcpp::Client<ServiceType>::SharedPtr client_;

    rclcpp::CallbackGroup::SharedPtr timerCb_callbackGroup;
    rclcpp::TimerBase::SharedPtr timer_;

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
