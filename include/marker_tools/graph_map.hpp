#ifndef H_MARKER_TOOLS_GRAPH_MAP_
#define H_MARKER_TOOLS_GRAPH_MAP_

#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <marker_tools/tree_map.hpp>

namespace potbot_lib{
    namespace marker_tools{
        
        using NodeId = std::string;

        template <typename T>
        struct GraphNode{
            NodeId id;
            std::vector<NodeId> connections;
            T data;
        };

        using VisualMarkerGraphNode = GraphNode<VisualMarker*>;

        class GraphMap : public TreeMap
        {
            private:

                rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_edges_;
                std::map<NodeId, VisualMarkerGraphNode> graph_;
                NodeId id_goal_node_, id_start_node_;

                rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
                rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

                void markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                // void addChild(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, const std::string child_name);

                void initializeMenu();
                void initializeMarker(std::string yaml_path = "");

                // void saveGraphMap(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, YAML::Node &marker_yaml);

                // std::vector<NodeId> breadthFirstSearch(const NodeId &start_id, const NodeId &goal_id);

                void publishGraphMap(const std::map<NodeId, VisualMarkerGraphNode> &graph_map, const std::vector<NodeId> &path = {});

            public:
                GraphMap(std::string name="marker", std::string node_namespace="");
                ~GraphMap(){};

                VisualMarkerGraphNode getNode(NodeId id);
        };
    }
}

#endif	// H_MARKER_TOOLS_GRAPH_MAP_