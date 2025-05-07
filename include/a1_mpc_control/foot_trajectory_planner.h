#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "quadruped_skate_mpc/msg/foot_position.hpp"



class FootTrajectoryPlanner : public rclcpp::Node {
public:
    FootTrajectoryPlanner();

private:
    void fsmCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishFootTrajectory();

    std::string current_state_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_sub_;
    rclcpp::Publisher<quadruped_skate_mpc::msg::FootPosition>::SharedPtr foot_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
