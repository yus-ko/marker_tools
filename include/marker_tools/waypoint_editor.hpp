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
                std::vector<VisualMarker> waypoints_;

                rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
                rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

                void markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

                void initializeMenu();
                void initializeMarker();

                std::vector<potbot_lib::Pose> interpolatePath(const std::vector<potbot_lib::Pose>& trajectory);

            public:
                WaypointEditor(std::string name="marker", std::string node_namespace="");
                ~WaypointEditor(){};

                std::vector<VisualMarker>* getVisualMarker();
        };
    }
}

#endif	// H_MARKER_TOOLS_TRAJECTORY_RECODER_