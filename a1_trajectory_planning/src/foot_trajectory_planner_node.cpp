#include "a1_mpc_control/foot_trajectory_planner.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootTrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}
