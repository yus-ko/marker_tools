#ifndef H_MARKER_TOOLS_TREE_MAP_
#define H_MARKER_TOOLS_TREE_MAP_

#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <potbot_ros/interactive_marker_manager.hpp>

namespace potbot_lib{
    namespace marker_tools{

        using NodeId = std::string;

        template <typename T>
        struct Node{
            NodeId id;
            NodeId parent;
            std::vector<NodeId> children;
            T data;
        };

        using VisualMarkerTreeNode = Node<VisualMarker*>;

        class TreeMap : public InteractiveMarkerManager
        {
            protected:

                rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;
                std::map<NodeId, VisualMarkerTreeNode> tree_;
                NodeId id_goal_node_, id_start_node_;

                virtual void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                virtual std::string duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual void deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                virtual void initializeMenu() override;
                virtual void initializeMarker(std::string yaml_path = "", bool set_default = true) override;
                virtual void initializeMarkerServer(const std::map<std::string, VisualMarker> &markers) override;

                virtual YAML::Node saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                virtual void setStartNode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                virtual void setGoalNode(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

                std::vector<NodeId> breadthFirstSearch(const NodeId &start_id, const NodeId &goal_id);

                void publishTreeMap(const std::map<NodeId, VisualMarkerTreeNode> &tree_map, const std::vector<NodeId> &path = {});

            public:
                TreeMap(std::string name="marker", std::string node_namespace="");
                ~TreeMap(){};

                // NodeId getStartNodeId() { return id_start_node_; };
                // NodeId getGoalNodeId() { return id_goal_node_; };

                VisualMarkerTreeNode getNode(NodeId id);
        };
    }
}

#endif	// H_MARKER_TOOLS_TREE_MAP_