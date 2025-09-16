#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <cmath>

class PatrolWithService : public rclcpp::Node {
    public:
        PatrolWithService() : Node("patrol_with_service"), direction_(0.0) {
            laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1));
            
            patrol_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&PatrolWithService::control_callback, this));

            client_ = this->create_client<robot_patrol::srv::GetDirection>("direction_service");
            
            RCLCPP_INFO(this->get_logger(), "Patrol node started");
        }
    
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr patrol_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

        sensor_msgs::msg::LaserScan::SharedPtr last_laser_;
        
        std::vector<float> front_scan_;
        double direction_;
        double angle_min_;
        double angle_increment_;

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            last_laser_ = msg;

            angle_min_ = msg->angle_min;
            angle_increment_ = msg->angle_increment;
            
            int scan_size = msg->ranges.size();
            int start_idx = (-M_PI_2 - angle_min_) / angle_increment_;
            int end_idx = (M_PI_2 - angle_min_) / angle_increment_;
            
            start_idx = std::max(0, std::min(start_idx, scan_size - 1));
            end_idx = std::max(0, std::min(end_idx, scan_size - 1));
            
            front_scan_.assign(msg->ranges.begin() + start_idx, msg->ranges.begin() + end_idx + 1);
        }

        bool obstacle_ahead() {
            if (front_scan_.empty()) return false;
            
            int scan_size = front_scan_.size();
            int center = scan_size / 2;
            int check_width = scan_size / 6;
            
            int start = center - check_width;
            int end = center + check_width;
            
            for (int i = start; i <= end; i++) {
                if (i >= 0 && i < scan_size && 
                    std::isfinite(front_scan_[i]) && 
                    front_scan_[i] < 0.35) {
                    return true;
                }
            }
            return false;
        }

        /*
        void find_safe_direction() {
            if (front_scan_.empty()) {
                direction_ = 0.0;
                return;
            }
            
            double max_distance = 0.0;
            int best_index = front_scan_.size() / 2;
            
            for (size_t i = 0; i < (front_scan_.size() - 1); i++) {
                if (std::isfinite(front_scan_[i]) && front_scan_[i] > max_distance) {
                    max_distance = front_scan_[i];
                    best_index = i;
                }
            }
            
            direction_ = -M_PI_2 + best_index * angle_increment_;
            
            direction_ = std::max(-M_PI_2, std::min(M_PI_2, direction_));
        }
        */

        void control_callback() {
            auto twist_msg = geometry_msgs::msg::Twist();
            
            twist_msg.linear.x = 0.1;
            twist_msg.angular.z = 0.0;

            patrol_pub_->publish(twist_msg);

            if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
                RCLCPP_WARN(this->get_logger(), "Direction Service not available");
                return;
            }

            if (obstacle_ahead()) {
                auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
                request->laser_data = *last_laser_;

                client_->async_send_request(
                    request,
                    [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future_response) {
                        auto response = future_response.get();
                        RCLCPP_INFO(this->get_logger(), "Service responded: %s",
                                    response->direction.c_str());

                        auto twist_msg = geometry_msgs::msg::Twist();

                        if (response->direction == "forward") {
                            twist_msg.linear.x = 0.1;
                            twist_msg.angular.z = 0.0;
                        } else if (response->direction == "left") {
                            twist_msg.linear.x = 0.1;
                            twist_msg.angular.z = 0.5;
                        } else if (response->direction == "right") {
                            twist_msg.linear.x = 0.1;
                            twist_msg.angular.z = -0.5;
                        }

                        patrol_pub_->publish(twist_msg);
                    });
            }
            /*
            if (obstacle_ahead()) {
                RCLCPP_INFO(this->get_logger(), "Obstacle Found!");
                auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
                request->laser_data = *last_laser_;

                std::string direction;

                auto result = client_->async_send_request(
                    request,
                    [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                            future_response) {
                    auto response = future_response.get();
                    RCLCPP_INFO(this->get_logger(), "Service responded: %s",
                                response->direction.c_str());
                    direction = response->direction;
                });
                
                if (direction == "forward") {
                    RCLCPP_INFO(this->get_logger(), "Going Forward!");
                    twist_msg.linear.x = 0.1;
                    twist_msg.angular.z = 0.0;
                } else if (direction == "left") {
                    RCLCPP_INFO(this->get_logger(), "Going Left!");
                    twist_msg.linear.x = 0.1;
                    twist_msg.angular.z = 0.5;
                } else if (direction == "right") {
                    RCLCPP_INFO(this->get_logger(), "Going Right!");
                    twist_msg.linear.x = 0.1;
                    twist_msg.angular.z = -0.5;
                }
            }
            */
                // find_safe_direction();
                // twist_msg.angular.z = direction_ / 2.0;
            
            // patrol_pub_->publish(twist_msg);
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrolWithService>());
    rclcpp::shutdown();
    return 0;
}