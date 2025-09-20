#include "rclcpp/rclcpp.hpp"
#include "marker_tools/tree_map.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        potbot_lib::marker_tools::TreeMap("tree_map").get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}