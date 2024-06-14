#include "rclcpp/rclcpp.hpp"

#include "citylab_interfaces/srv/get_direction.hpp"

#include <functional>
#include <memory>

using namespace std::placeholders;

using ServiceType = citylab_interfaces::srv::GetDirection;


class DirectionService : public rclcpp::Node
{
public:
    DirectionService()
        : Node{ "DirectionService" }
        , srv_ { create_service<ServiceType>("direction_service", std::bind(&DirectionService::directionCb, this, _1, _2)) }
    {
        RCLCPP_INFO(this->get_logger(), "Constructing Direction Service...");

        RCLCPP_INFO(this->get_logger(), "Direction Service Constructed");
    }

private:
    void directionCb(const std::shared_ptr<ServiceType::Request>  request,
                     const std::shared_ptr<ServiceType::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Direction service callback triggered");


    }

private:
    rclcpp::Service<ServiceType>::SharedPtr srv_;

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto direction_srv_node = std::make_shared<DirectionService>();

    RCLCPP_INFO(direction_srv_node->get_logger(), "Running Direction Service Main");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(direction_srv_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
