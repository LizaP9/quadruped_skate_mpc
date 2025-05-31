#include "a1_mpc_control/foot_trajectory_planner.h"

FootTrajectoryPlanner::FootTrajectoryPlanner()
: Node("foot_trajectory_planner")
{
    fsm_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/a1/fsm", 10,
        std::bind(&FootTrajectoryPlanner::fsmCallback, this, std::placeholders::_1));

    foot_pub_ = this->create_publisher<a1_msgs::msg::FootPosition>(
        "/a1/foot_des_positions", 10); 


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FootTrajectoryPlanner::publishFootTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "FootTrajectoryPlanner initialized.");

    start_time_ = this->now();
}

void FootTrajectoryPlanner::fsmCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // current_state_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "FSM state updated to: %s", current_state_.c_str());

    last_state_ = current_state_;
    current_state_ = msg->data;

    if (last_state_ != "PREPARE_PUSH" && current_state_ == "PREPARE_PUSH") {
        start_time_ = this->now();
    }
}

void FootTrajectoryPlanner::publishFootTrajectory()
{
    a1_msgs::msg::FootPosition foot_msg;
    foot_msg.header.stamp = this->now();
    foot_msg.header.frame_id = "trunk";

    // current_state_ = "PREPARE_PUSH";

    if (current_state_ == "STAND_ON_BOARD") {
        foot_msg.fr.x = 0.183;
        foot_msg.fr.y = -0.03;
        foot_msg.fr.z = -0.3804;

        foot_msg.fl.x = 0.183;
        foot_msg.fl.y = 0.03;
        foot_msg.fl.z = -0.3804;

        foot_msg.rr.x = -0.183;
        foot_msg.rr.y = -0.03;
        foot_msg.rr.z = -0.3804;

        foot_msg.rl.x = -0.183;
        foot_msg.rl.y = 0.03;
        foot_msg.rl.z = -0.3804;
    }
    else if (current_state_ == "PREPARE_PUSH") {


        foot_msg.fl.x = 0.183;
        foot_msg.fl.y = 0.03;
        foot_msg.fl.z = -0.33;

        foot_msg.rr.x = -0.183;
        foot_msg.rr.y = -0.03;
        foot_msg.rr.z = -0.33;

        foot_msg.rl.x = -0.183;
        foot_msg.rl.y = 0.03;
        foot_msg.rl.z = -0.33;

        double t = (this->now() - start_time_).seconds();
        double duration = 2;  // seconds
        double alpha = std::min(t / duration, 1.0);

        // Начальная и конечная точки в плоскости YZ
        double y0 = -0.03, z0 = -0.33;
        double y2 = -0.08, z2 = -0.4104;

        // Точка подъема (в середине по Y, выше по Z)
        double y1 = (y0 + y2) / 2.0;
        double z1 = std::max(z0, z2) + 0.05;  // подъем на 5см выше самой верхрхней точки

        // Применяем траекторию
        foot_msg.fr.x = 0.183;  // x постоянен
        foot_msg.fr.y = bezier(y0, y1, y2, alpha);
        foot_msg.fr.z = bezier(z0, z1, z2, alpha);



    }
    else if (current_state_ == "RETURN_TO_BOARD") {
        foot_msg.fr.x = 0.2;
        foot_msg.fr.y = -0.1;
        foot_msg.fr.z = -0.33;

        foot_msg.fl.x = 0.2;
        foot_msg.fl.y = 0.1;
        foot_msg.fl.z = -0.33;

        foot_msg.rr.x = -0.2;
        foot_msg.rr.y = -0.1;
        foot_msg.rr.z = -0.33;

        foot_msg.rl.x = -0.2;
        foot_msg.rl.y = 0.1;
        foot_msg.rl.z = -0.33;
    }
    else {
        return;
    }

    foot_pub_->publish(foot_msg);
}


double FootTrajectoryPlanner::bezier(double p0, double p1, double p2, double t) {
    return (1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * p1 + t * t * p2;
}