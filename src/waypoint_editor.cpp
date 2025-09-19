#include <marker_tools/waypoint_editor.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		WaypointEditor::WaypointEditor(std::string name, std::string node_namespace) : InteractiveMarkerManager(name,node_namespace)
		{
			initializeMenu();
			initializeMarker();
			
			pub_waypoints_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 1);

			// srv_save_marker_trajectory_ = pnh.advertiseService("save_marker_tarajectory", &InteractiveMarkerManager::serviceSaveMarkerTrajectory, this);
			// srv_clear_marker_trajectory_ = pnh.advertiseService("clear_marker_tarajectory", &InteractiveMarkerManager::serviceClearMarkerTrajectory, this);
			
			RCLCPP_INFO(this->get_logger(), "TrajectoryRecoder initialized");
		}

		// void TrajectoryRecoder::initializeParameter()
		// {
		// 	this->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		// 	this->declare_parameter("marker_yaml_path", rclcpp::ParameterValue("interactive_markers.yaml"));
		// }

		void WaypointEditor::initializeMenu()
		{
			
		}

		void WaypointEditor::initializeMarker()
		{
			function_change_position_ = [this](
				const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
					changePosition(feedback);
					markerFeedback(feedback);
				};
			function_change_rotation_ = [this](
				const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
					changeRotation(feedback);
					markerFeedback(feedback);
				};
			
			for (const auto &cm:controllable_markers_)
				registerFeedback(cm.first, function_change_position_);
			
		}

		rcl_interfaces::msg::SetParametersResult WaypointEditor::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
		{
			RCLCPP_INFO(this->get_logger(), "parameter changed");
			auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
			results->successful = true;

			return *results;
		}

		void WaypointEditor::markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			// RCLCPP_INFO(this->get_logger(),"markerFeedback");
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				std::vector<potbot_lib::Pose> waypoints;
				for (const auto &cm:controllable_markers_)
					waypoints.push_back(potbot_lib::utility::get_pose(cm.second.marker.pose));
				auto path = interpolatePath(waypoints);

				nav_msgs::msg::Path msg;
				potbot_lib::utility::to_msg(waypoints, msg);
				msg.header.frame_id = frame_id_global_;
				msg.header.stamp = this->get_clock()->now();
				pub_waypoints_->publish(msg);
			}
		}

		std::vector<potbot_lib::Pose> WaypointEditor::interpolatePath(const std::vector<potbot_lib::Pose>& trajectory)
		{	
			auto points_num = trajectory.size();
			if (points_num < 2)
			{
				return trajectory;
			}

			size_t interpolate_num_limit = 100;
			static size_t last_interpolated_index = 0;
			std::vector<Pose> no_interpolate, to_interpolate, interpolated, out;
			if (points_num > interpolate_num_limit)
			{
				to_interpolate = std::vector<Pose>(trajectory.end() - interpolate_num_limit, trajectory.end());
				no_interpolate = std::vector<Pose>(trajectory.begin(), trajectory.end() - interpolate_num_limit);
			}
			else
				to_interpolate = trajectory;

			interpolate::bezier(to_interpolate, to_interpolate.size()*100, interpolated);
			// interpolated.insert(trajectory.begin(), no_interpolate.begin(), no_interpolate.end());
			for (const auto& p:no_interpolate)
				out.push_back(p);
			for (const auto& p:interpolated)
				out.push_back(p);
			RCLCPP_INFO(this->get_logger(), "%d, %d", int(to_interpolate.size()), int(no_interpolate.size()));
		
			// // std::vector<Eigen::Affine3d> traj_vecs = utility::get_vec(split_traj);
			// if (visual_markers_[id].trajectory_interpolation_method == "spline")
			// {
			// 	// interpolate::spline(traj_vecs, traj_vecs.size(), traj_vecs);
			// 	interpolate::bezier(split_traj, split_traj.size(), interpolated);
			// }
			// else if (visual_markers_[id].trajectory_interpolation_method == "bezier")
			// {
			// 	interpolate::bezier(split_traj, split_traj.size(), interpolated);
			// }
			return out;
		}

		// std::vector<VisualMarker>* WaypointEditor::getVisualMarker()
		// {
		// 	return &visual_markers_;
		// }
	}
}