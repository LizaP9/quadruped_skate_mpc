#include "a1_mpc_control/foot_trajectory_planner.h"

FootTrajectoryPlanner::FootTrajectoryPlanner()
: Node("foot_trajectory_planner")
{
    fsm_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/a1/fsm", 10,
        std::bind(&FootTrajectoryPlanner::fsmCallback, this, std::placeholders::_1));

    foot_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/a1/foot_des_positions", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FootTrajectoryPlanner::publishFootTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "FootTrajectoryPlanner initialized.");
}

void FootTrajectoryPlanner::fsmCallback(const std_msgs::msg::String::SharedPtr msg)
{
    current_state_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "FSM state updated to: %s", current_state_.c_str());
}

void FootTrajectoryPlanner::publishFootTrajectory()
{
    geometry_msgs::msg::PointStamped foot_msg;
    foot_msg.header.stamp = this->now();
    foot_msg.header.frame_id = "base";

    if (current_state_ == "STAND" || current_state_ == "STAND_ON_BOARD") {
        foot_msg.point.x = 0.2;
        foot_msg.point.y = 0.1;
        foot_msg.point.z = -0.33;
    }
    else if (current_state_ == "PREPARE_PUSH") {
        foot_msg.point.x = 0.3;
        foot_msg.point.y = -0.2;
        foot_msg.point.z = -0.4;
    }
    else if (current_state_ == "RETURN_TO_BOARD") {
        foot_msg.point.x = 0.2;
        foot_msg.point.y = 0.1;
        foot_msg.point.z = -0.33;
    }
    else {
        return;  // Не публикуем в IDLE, LAY и других
    }

    foot_pub_->publish(foot_msg);
}
