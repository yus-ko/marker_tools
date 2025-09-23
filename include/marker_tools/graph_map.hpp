#ifndef H_MARKER_TOOLS_GRAPH_MAP_
#define H_MARKER_TOOLS_GRAPH_MAP_

#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <marker_tools/tree_map.hpp>

namespace potbot_lib{
    namespace marker_tools{

        template <typename T>
        struct GraphNode{
            NodeId id;
            std::set<NodeId> connections;
            T data;
        };

        using VisualMarkerGraphNode = GraphNode<VisualMarker*>;

        class GraphMap : public TreeMap
        {
            protected:
                std::shared_ptr<interactive_markers::MenuHandler>
                    menu_handler_connect_, 
                    menu_handler_cancel_;
                std::map<NodeId, VisualMarkerGraphNode> graph_;

                NodeId connection_source_;

                virtual void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                virtual void initializeMenu() override;
                virtual void initializeMarker(std::string yaml_path = "", bool set_default = true) override;
                virtual void initializeMarkerServer(const std::map<std::string, VisualMarker> &markers, const std::vector<std::string> &marker_with_controller = {}) override;

                virtual YAML::Node saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual std::string duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual void deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                virtual void setStartNode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual void setGoalNode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                void changeToConnectMode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                void changeToNormalMode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                void connectNode(const NodeId &source, const NodeId &target);

                std::vector<NodeId> getPath(const NodeId &start, const NodeId &goal);

                void publishGraphMap(const std::map<NodeId, VisualMarkerGraphNode> &graph_map, const std::vector<NodeId> &path = {});

            public:
                GraphMap(std::string name="marker", std::string node_namespace="");
                ~GraphMap(){};

                VisualMarkerGraphNode getNode(NodeId id);
        };
    }
}

#endif	// H_MARKER_TOOLS_GRAPH_MAP_