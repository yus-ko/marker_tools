#include "rclcpp/rclcpp.hpp"
#include "marker_tools/trajectory_recoder.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = potbot_lib::marker_tools::TrajectoryRecoder("trajectory");
    node.configure();
    node.activate();
    rclcpp::spin(node.get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}