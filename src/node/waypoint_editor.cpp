#include "rclcpp/rclcpp.hpp"
#include "marker_tools/waypoint_editor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        potbot_lib::marker_tools::WaypointEditor("waypoint").get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}