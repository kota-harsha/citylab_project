#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TestService : public rclcpp::Node
{
public:
    TestService() : Node("test_service"), request_sent_(false) {
        client_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service");
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&TestService::laser_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Test Service Node Ready");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (request_sent_){
            return;
        }

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;

        request_sent_ = true;

        auto result = client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                    future_response) {
            auto response = future_response.get();
            RCLCPP_INFO(this->get_logger(), "Service responded: %s",
                        response->direction.c_str());

            rclcpp::shutdown();
        });
    }
    
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    bool request_sent_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestService>());
    rclcpp::shutdown();
    return 0;
}