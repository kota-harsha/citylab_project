#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "robot_patrol/action/go_to_pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose() : Node("go_to_pose_action") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;

  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D desired_pos_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f, %.2f)",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> /*goal_handle*/) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    auto goal = goal_handle->get_goal();
    desired_pos_ = goal->goal_pos;

    rclcpp::Rate rate(10);
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();

    while (rclcpp::ok()) {
      float dx = desired_pos_.x - current_pos_.x;
      float dy = desired_pos_.y - current_pos_.y;
      float distance = std::sqrt(dx * dx + dy * dy);
      float angle_to_goal = std::atan2(dy, dx);
      float angle_error = angle_to_goal - current_pos_.theta;

      geometry_msgs::msg::Twist cmd_vel;

      if (std::fabs(angle_error) > 0.1) {
        cmd_vel.angular.z = angle_error;
        cmd_vel.linear.x = 0.0;
      } else {
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.0;
      }

      cmd_vel_pub_->publish(cmd_vel);

      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);

      if (distance < 0.1) {
        break;
      }

      rate.sleep();
    }

    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);

    auto result = std::make_shared<GoToPoseAction::Result>();
    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPose>());
  rclcpp::shutdown();
  return 0;
}