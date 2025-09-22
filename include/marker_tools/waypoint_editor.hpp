#ifndef H_MARKER_TOOLS_WAYPOINT_EDITOR_
#define H_MARKER_TOOLS_WAYPOINT_EDITOR_

#include <rclcpp/rclcpp.hpp>
#include <potbot_ros/interactive_marker_manager.hpp>

namespace potbot_lib{
    namespace marker_tools{

        class WaypointEditor : public InteractiveMarkerManager
        {
            private:

                rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_waypoints_;
                std::vector<VisualMarker*> waypoints_;

                void initializeMarker(std::string yaml_path = "", bool set_default = true) override;

                void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                YAML::Node saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                std::string duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                void deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                std::vector<potbot_lib::Pose> interpolatePath(const std::vector<potbot_lib::Pose>& trajectory);

                void publishWaypointPath(const std::vector<VisualMarker*> &waypoints);

            public:
                WaypointEditor(std::string name="marker", std::string node_namespace="");
                ~WaypointEditor(){};
        };
    }
}

#endif	// H_MARKER_TOOLS_TRAJECTORY_RECODER_