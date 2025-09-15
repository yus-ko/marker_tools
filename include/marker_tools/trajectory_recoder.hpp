#ifndef H_MARKER_TOOLS_TRAJECTORY_RECODER_
#define H_MARKER_TOOLS_TRAJECTORY_RECODER_

#include <rclcpp/rclcpp.hpp>
#include <potbot_ros/interactive_marker_manager.hpp>

namespace potbot_lib{
    namespace marker_tools{

        typedef struct{
            std::vector<potbot_lib::Pose> trajectory;
            std::vector<std_msgs::msg::Header> headers;
            bool trajectory_recording = false;
            u_int8_t trajectory_marker_type = visualization_msgs::msg::Marker::LINE_STRIP;
            std::string trajectory_interpolation_method = "none";
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;

            nav_msgs::msg::Path to_msg()
            {
                nav_msgs::msg::Path msg;
                msg.header = headers.back();
                msg.poses.resize(headers.size());
                for (size_t i = 0; i < headers.size(); i++)
                {
                    msg.poses[i].header = headers[i];
                    msg.poses[i].pose = potbot_lib::utility::get_pose(trajectory[i]);
                }
                return msg;
            };

            void publish() {publisher->publish(to_msg());};
        } TrajectoryInfo;

        class TrajectoryRecoder : public InteractiveMarkerManager
        {
            private:

                // ros::ServiceServer srv_save_marker_trajectory_, srv_clear_marker_trajectory_;

                size_t interactive_marker_num_ = 1;
                std::map<std::string, TrajectoryInfo> trajectory_;

                rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
                rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

                void markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

                void initializeMenu();
                void initializeMarker();

                void interpolateTrajectory(size_t id);

                void clearTrajectory(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

            public:
                TrajectoryRecoder(std::string name="marker", std::string node_namespace="");
                ~TrajectoryRecoder(){};

                std::vector<VisualMarker>* getVisualMarker();
        };
    }
}

#endif	// H_MARKER_TOOLS_TRAJECTORY_RECODER_