#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Patrol : public rclcpp::Node {
    public:
        Patrol() : Node("patrolling_node")
        {
            patrol_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1)
            );
            timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Patrol::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "Patrol node started");
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr patrol_pub;
        rclcpp::TimerBase::SharedPtr timer;

        // sensor_msgs::msg::LaserScan::SharedPtr data;
        // geometry_msgs::msg::Twist twist_msg;

        double direction_;

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr data){
            bool obstacle_detected = false;

            int num_readings = data->ranges.size();
            int front_start = 3 * num_readings / 8;
            int front_end = 5 * num_readings / 8;

            for (int i = front_start; i <= front_end; i++) {
                if (data->ranges[i] < 0.35 && data->ranges[i] > data->range_min) {
                    obstacle_detected = true;
                    // RCLCPP_INFO(this->get_logger(), "Obstacle_Detected");
                    break;
                }
            }

            front_start = num_readings / 4;
            front_end = 3 * num_readings / 4;

            if (obstacle_detected) {
                // int start_index = 0;
                // int end_index   = (int)data->ranges.size()-1;

                float max_dist = -1.0;
                int best_index = front_start;

                // RCLCPP_INFO(this->get_logger(), "Data Ranges start (Left): %f", data->ranges[front_start]);
                // RCLCPP_INFO(this->get_logger(), "Data Ranges end (Right): %f", data->ranges[front_end]);

                for (int i = front_start; i <= front_end; i++) {
                    float r = data->ranges[i];
                    if (std::isfinite(r) && r >= data->range_min && r <= data->range_max) {
                        if (r > max_dist) {
                            max_dist = r;
                            best_index = i;
                        }
                    }
                }

                /*
                if (best_index >= front_start && best_index < num_readings/2){
                    RCLCPP_INFO(this->get_logger(), "Move Left");
                    this->direction_ = 0.8;
                
                }
                else if (best_index > num_readings/2 && best_index <= front_end) {
                    RCLCPP_INFO(this->get_logger(), "Move Right");
                    this->direction_ = -0.8;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Stay Center");
                    this->direction_ = 0.0;
                }
                */

                this->direction_ = (data->angle_min) + best_index * data->angle_increment;
                // RCLCPP_INFO(this->get_logger(), "Direction: %f", direction_);

            }
            else {
                this->direction_ = 0.0;
            }
        }

        void timer_callback() {
            // RCLCPP_INFO(this->get_logger(), "Timer Callback Called");
            auto twist_msg = geometry_msgs::msg::Twist();

            twist_msg.linear.x = 0.1;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = this->direction_ / 2.0;
            
            patrol_pub->publish(twist_msg);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}