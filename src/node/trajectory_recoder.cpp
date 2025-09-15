#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_thread.hpp"
#include "marker_tools/trajectory_recoder.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        potbot_lib::marker_tools::TrajectoryRecoder("trajectory").get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}