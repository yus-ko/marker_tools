#ifndef H_MARKER_TOOLS_WAYPOINT_EDITOR_
#define H_MARKER_TOOLS_WAYPOINT_EDITOR_

#include <rclcpp/rclcpp.hpp>
#include <potbot_ros/interactive_marker_manager.hpp>

namespace potbot_lib{
    namespace marker_tools{

        class WaypointEditor : public InteractiveMarkerManager
        {
            protected:

                rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_waypoints_;
                std::vector<VisualMarker*> waypoints_;
                bool is_interpolate_ = false;

                virtual void initializeMenu() override;
                virtual void initializeMarker(std::string yaml_path = "", bool set_default = true) override;
                virtual void initializeMarkerServer(const std::map<std::string, VisualMarker> &markers, const std::vector<std::string> &marker_with_controller = {}) override;

                virtual void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual YAML::Node saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual std::string duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;
                virtual void deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                void setInterpolate(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                std::vector<potbot_lib::Pose> interpolatePath(const std::vector<potbot_lib::Pose>& trajectory);

                void publishWaypointPath(const std::vector<VisualMarker*> &waypoints);

                nav_msgs::msg::Path getMsg(const std::vector<VisualMarker*> &waypoints);

            public:
                WaypointEditor(std::string name="marker", std::string node_namespace="");
                ~WaypointEditor(){};

                nav_msgs::msg::Path getWaypoints();
        };
    }
}

#endif	// H_MARKER_TOOLS_TRAJECTORY_RECODER_