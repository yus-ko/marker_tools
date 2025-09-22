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

                std::map<std::string, TrajectoryInfo> trajectory_;

                void changePosition(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) override;

                void initializeMenu() override;
                void initializeMarker(std::string yaml_path = "", bool set_default = true) override;

                std::vector<potbot_lib::Pose> interpolateTrajectory(const std::vector<potbot_lib::Pose>& trajectory);

                void clearTrajectory(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
                void saveTrajectory(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string type = "csv");
                bool saveCSV(std::string marker_name, std::string file_path);
                bool saveYAML(std::string marker_name, std::string file_path);

            public:
                TrajectoryRecoder(std::string name="marker", std::string node_namespace="");
                ~TrajectoryRecoder(){};

                std::vector<VisualMarker>* getVisualMarker();
        };
    }
}

#endif	// H_MARKER_TOOLS_TRAJECTORY_RECODER_