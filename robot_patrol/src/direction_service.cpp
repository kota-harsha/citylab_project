#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <memory>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
    public:
        DirectionService() : Node("direction_service"), direction_("forward") {
            srv_ = create_service<robot_patrol::srv::GetDirection> (
                "direction_service", std::bind(&DirectionService::direction_callback, this, _1, _2)
            );
        }

    private:
        rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr srv_;
        std::string direction_;

        float total_dist_sec_right = 0.0;
        float total_dist_sec_front = 0.0;
        float total_dist_sec_left = 0.0;
        
        sensor_msgs::msg::LaserScan::SharedPtr input;

        void direction_callback (
            const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
            const std::shared_ptr<robot_patrol::srv::GetDirection::Response> response
        ){
            auto input = std::make_shared<sensor_msgs::msg::LaserScan>(request->laser_data);
            int num_readings = input->ranges.size();

            int start = num_readings / 4;
            int end = (3 * num_readings) / 4;

            int interval = (int)((end - start)/3);

            int right = start + interval;
            int left = start + (2*interval);

            for (int i = start; i < end; i++){
                if (input->ranges[i] <= input->range_max) {
                    if (i < right) {
                        total_dist_sec_right += input->ranges[i];
                    }
                    else if (i > left) {
                        total_dist_sec_left += input->ranges[i];
                    }
                    else {
                        total_dist_sec_front += input->ranges[i];
                    }
                }
            }

            float max = std::max({total_dist_sec_front, total_dist_sec_left, total_dist_sec_right});

            if (max == total_dist_sec_left) {
                direction_ = "left";
            }
            else if (max == total_dist_sec_right) {
                direction_ = "right";
            }
            else {
                direction_ = "forward";
            }

            response->direction = direction_;
            RCLCPP_INFO(this->get_logger(), "Service finished \n");
        }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<DirectionService>();

    // rcutils_logging_set_logger_level(server_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    // RCLCPP_DEBUG(server_node->get_logger(), "SERVICE = /direction_service");
    
    rclcpp::spin(server_node);
    rclcpp::shutdown();
    return 0;
}
