#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node"), direction_(0.0) {
        // Create subscriber and publisher
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
        
        patrol_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create timer for 10Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Patrol node started");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr patrol_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<float> front_scan_;
    double direction_;
    double angle_min_;
    double angle_increment_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
        
        // Check front 60° (-30° to +30°) for obstacles
        int scan_size = front_scan_.size();
        int center = scan_size / 2;
        int check_width = scan_size / 6;  // 60° out of 180°
        
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

    void find_safe_direction() {
        if (front_scan_.empty()) {
            direction_ = 0.0;
            return;
        }
        
        // Find direction with maximum distance
        double max_distance = 0.0;
        int best_index = front_scan_.size() / 2;  // Default to center
        
        for (size_t i = 0; i < front_scan_.size(); i++) {
            if (std::isfinite(front_scan_[i]) && front_scan_[i] > max_distance) {
                max_distance = front_scan_[i];
                best_index = i;
            }
        }
        
        // Convert index to angle
        direction_ = -M_PI_2 + best_index * angle_increment_;
        
        // Limit to front hemisphere
        direction_ = std::max(-M_PI_2, std::min(M_PI_2, direction_));
    }

    void control_callback() {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Always move forward
        twist_msg.linear.x = 0.1;
        
        // Check for obstacles and set angular velocity
        if (obstacle_ahead()) {
            find_safe_direction();
            twist_msg.angular.z = direction_ / 2.0;
        } else {
            twist_msg.angular.z = 0.0;  // Go straight
        }
        
        patrol_pub_->publish(twist_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}