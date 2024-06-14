#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "citylab_interfaces/srv/get_direction.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

using ServiceType = citylab_interfaces::srv::GetDirection;


class TestService : public rclcpp::Node
{
public:
    TestService()
        : Node{ "TestService" }
        , laserScanCb_callbackGroup{ create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
        , client_{ create_client<ServiceType>("direction_service") }
    {
        RCLCPP_INFO(get_logger(), "Constructing TestService..");

        rclcpp::SubscriptionOptions laserSubOptions;
        laserSubOptions.callback_group = laserScanCb_callbackGroup;
        laserSub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
            std::bind(&TestService::laserScanCb, this, _1), laserSubOptions);

        RCLCPP_INFO(get_logger(), "TestService Constructed");
    }

private:
    void laserScanCb(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto request = std::make_shared<ServiceType::Request>();
        request->laser_data = *msg;

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "direction: %s", result.get()->direction.c_str());
    }

private:
    rclcpp::CallbackGroup::SharedPtr laserScanCb_callbackGroup;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;

    rclcpp::Client<ServiceType>::SharedPtr client_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto test_service_node = std::make_shared<TestService>();

    RCLCPP_INFO(test_service_node->get_logger(), "Running Test Service Main");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_service_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
