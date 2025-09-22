#include "rclcpp/rclcpp.hpp"
#include "marker_tools/waypoint_editor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = potbot_lib::marker_tools::WaypointEditor("waypoint");
    node.configure();
    node.activate();
    rclcpp::spin(node.get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}