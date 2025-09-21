#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_thread.hpp"
#include "potbot_ros/interactive_marker_manager.hpp"

using namespace std::chrono_literals;

class MarkerNode : public rclcpp::Node
{
public:
    MarkerNode() : Node("marker_node")
    {
        // node_ = std::make_shared<rclcpp::Node>("marker_node2");
        imm_ = std::make_shared<potbot_lib::InteractiveMarkerManager>();
        // imm_->configure();
        // imm_->activate();
        imm_thread_ = std::make_unique<nav2_util::NodeThread>(imm_);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&MarkerNode::timer_callback, this));
    }

private:
    void timer_callback(){
        // auto pose = imm_->getMarkerPose("marker_0");
        // RCLCPP_INFO(this->get_logger(), "%.2f, %.2f", pose.position.x, pose.position.y);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<potbot_lib::InteractiveMarkerManager> imm_;

    std::unique_ptr<nav2_util::NodeThread> imm_thread_;
};

int main(int argc, char * argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MarkerNode>());
    // rclcpp::shutdown();

    // rclcpp::init(argc, argv);
    // auto node_marker = std::make_shared<MarkerNode>();
    // // auto node_imm = std::make_shared<potbot_lib::InteractiveMarkerManager>("marker");
    // // ln->configure();
    // // ln->activate();
    // rclcpp::executors::SingleThreadedExecutor exe;
    // exe.add_node(node_marker->get_node_base_interface());
    // // exe.add_node(node_imm->get_node_base_interface());
    // exe.spin();
    // rclcpp::shutdown();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<potbot_lib::InteractiveMarkerManager>("marker");
    node->configure();
    node->activate();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}