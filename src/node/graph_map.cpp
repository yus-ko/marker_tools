#include "rclcpp/rclcpp.hpp"
#include "marker_tools/graph_map.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = potbot_lib::marker_tools::GraphMap("graph_map");
    node.configure();
    node.activate();
    rclcpp::spin(node.get_node_base_interface());
    // node.deactivate();
    // node.cleanup();
    rclcpp::shutdown();
    return 0;
}