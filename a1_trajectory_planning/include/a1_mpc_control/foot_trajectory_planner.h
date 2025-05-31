#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "a1_msgs/msg/foot_position.hpp"



class FootTrajectoryPlanner : public rclcpp::Node {
public:
    FootTrajectoryPlanner();

private:
    void fsmCallback(const std_msgs::msg::String::SharedPtr msg);
    void publishFootTrajectory();
    double bezier(double p0, double p1, double p2, double t);

    std::string current_state_;
    std::string last_state_;
    rclcpp::Time start_time_;



    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_sub_;
    rclcpp::Publisher<a1_msgs::msg::FootPosition>::SharedPtr foot_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
