#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "citylab_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"

#include <functional>
#include <memory>
#include <numeric>

using namespace std::placeholders;

using ServiceType = citylab_interfaces::srv::GetDirection;


class DirectionService : public rclcpp::Node
{
public:
    DirectionService()
        : Node{ "DirectionService" }
        , logger_{ get_logger() }
        , srv_ { create_service<ServiceType>("direction_service", std::bind(&DirectionService::directionCb, this, _1, _2)) }
    {
        RCLCPP_INFO(logger_, "Constructing Direction Service...");

        RCLCPP_INFO(logger_, "Direction Service Constructed");
    }

private:
    void directionCb(const std::shared_ptr<ServiceType::Request>  request,
                     const std::shared_ptr<ServiceType::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Direction service callback triggered");

        RCLCPP_INFO(logger_, "minAngle: %f, maxAngle: %f, angleInc: %f, ",
            request->laser_data.angle_min, request->laser_data.angle_max, request->laser_data.angle_increment);

        gather_laser_data(request->laser_data);

        const auto &rng = request->laser_data.ranges;
        const auto begIt{ rng.cbegin() };
        const auto leftIter{ std::next(begIt, left_idx_) };
        const auto leftFrontIter{ std::next(begIt, left_front_idx_) };
        const auto frontIter{ std::next(begIt, front_idx_) };
        const auto rightFrontIter{ std::next(begIt, right_front_idx_) };
        const auto rightIter{ std::next(begIt, right_idx_) };
        
        auto filterOutOfRange = [rMin = range_min_, rMax = range_max_](float total, float b)
        {
            return total + (b < rMin ? 0 : b > rMax ? 0 : b);
        };

        const auto total_dist_sec_left  = std::accumulate(leftFrontIter, leftIter, 0.0f, filterOutOfRange);
        const auto total_dist_sec_front = std::accumulate(rightFrontIter, leftFrontIter, 0.0f, filterOutOfRange);
        const auto total_dist_sec_right = std::accumulate(rightIter, rightFrontIter, 0.0f, filterOutOfRange);

        RCLCPP_INFO(logger_, "left: %f, front: %f, right: %f, ", *leftIter, *frontIter, *rightIter);
        RCLCPP_INFO(logger_, "totals: left: %f, front: %f, right: %f, ", total_dist_sec_left, total_dist_sec_front, total_dist_sec_right);

        const auto maxTotal = std::max({total_dist_sec_front, total_dist_sec_left, total_dist_sec_right});

        response->direction = "forward";
        if (maxTotal == total_dist_sec_left)
        {
            response->direction = "left";
        }
        else if (maxTotal == total_dist_sec_right)
        {
            response->direction = "right";
        }
    }

    void gather_laser_data(const sensor_msgs::msg::LaserScan &laser_data)
    {
        range_min_ = laser_data.range_min;
        range_max_ = laser_data.range_max;

        const auto nRanges = laser_data.ranges.size();

        // The Left and Right seems to be swapped. I'm not entirely sure why.
        // Could the laser be installed upside down?
        front_idx_       = nRanges / 2;                 //  0 degrees, 0 radians
        right_idx_       = nRanges / 4;                 // -90 degrees, -pi/2
        right_front_idx_ = right_idx_ + nRanges / 6;    // -30 degrees, -pi/6
        left_idx_        = (nRanges / 4) * 3;           // +90 degrees, +pi/2
        left_front_idx_  = left_idx_ - nRanges / 6;     // +30 degrees, +pi/6

        RCLCPP_INFO(get_logger(), "# ranges: %zu, lIdx: %zu, lfIdx: %zu, fIdx: %zu, frIdx: %zu, rIdx: %zu",
            nRanges, left_idx_, left_front_idx_, front_idx_, right_front_idx_, right_idx_);

        RCLCPP_INFO(get_logger(), "rMIn: %f, rMax: %f", range_min_, range_max_);
    }

private:
    rclcpp::Logger logger_;
    rclcpp::Service<ServiceType>::SharedPtr srv_;

    float  range_min_{};
    float  range_max_{};

    size_t front_idx_{};
    size_t left_idx_{};
    size_t left_front_idx_{};
    size_t right_idx_{};
    size_t right_front_idx_{};

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
