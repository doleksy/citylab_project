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
        RCLCPP_INFO(get_logger(), "Constructing Direction Service...");

        RCLCPP_INFO(get_logger(), "Direction Service Constructed");
    }

private:
    void directionCb(const std::shared_ptr<ServiceType::Request>  request,
                     const std::shared_ptr<ServiceType::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Direction service callback triggered");

        float angleMin = request->laser_data.angle_min;
        float angleMax = request->laser_data.angle_max;
        float angleInc = request->laser_data.angle_increment;

        float rangeMin = request->laser_data.range_min;
        float rangeMax = request->laser_data.range_max;

        auto nRanges = request->laser_data.ranges.size();

        RCLCPP_INFO(get_logger(), "aMin: %f, aMax: %f, aInc: %f", angleMin, angleMax, angleInc);
        RCLCPP_INFO(get_logger(), "rMIn: %f, rMax: %f, # ranges: %zu", rangeMin, rangeMax, nRanges);

        response->direction = "testing";
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
