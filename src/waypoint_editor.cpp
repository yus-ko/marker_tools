#include <marker_tools/waypoint_editor.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		WaypointEditor::WaypointEditor(std::string name, std::string node_namespace)
			: InteractiveMarkerManager(name,node_namespace)
		{	
			pub_waypoints_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 1);
		}

		void WaypointEditor::initializeMenu()
		{
			InteractiveMarkerManager::initializeMenu();

			entry_handles_["interpolate"] = menu_handler_->insert("interpolate", 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback){
					setInterpolate(feedback);});
			menu_handler_->setCheckState(entry_handles_["interpolate"], 
				interactive_markers::MenuHandler::CheckState::UNCHECKED);
		}

		void WaypointEditor::initializeMarker(std::string yaml_path, bool set_default)
		{
			waypoints_.clear();

			try 
			{
				
				YAML::Node root = YAML::LoadFile(yaml_path);
				if (root["markers"]) 
				{
					for (const auto& node : root["markers"]) 
					{
						auto vm = getVisualMarker(node);
						addMarker(vm);
						waypoints_.push_back(&controllable_markers_[vm.marker.name]);
					}

					RCLCPP_INFO(this->get_logger(), "Waypoints loaded: %s", yaml_path.c_str());
				}
			} 
			catch (const std::exception& e) 
			{
				RCLCPP_INFO(this->get_logger(), "Failed to waypoint load. Marker yaml: %s", e.what());

				if (set_default)
				{
					addMarker(default_visual_marker_.text);
					waypoints_.push_back(&controllable_markers_[default_visual_marker_.text]);
					RCLCPP_INFO(this->get_logger(), "Set to default");
				}
			}
		}

		void WaypointEditor::initializeMarkerServer(const std::map<std::string, VisualMarker> &markers)
		{
			InteractiveMarkerManager::initializeMarkerServer(markers);
			publishWaypointPath(waypoints_);
		}

		YAML::Node WaypointEditor::saveMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			YAML::Node root;
			YAML::Node node = root["markers"];

			for (const auto& wp:waypoints_)
				node.push_back(getYamlNode(*wp));

			std::string yaml_path = marker_file_;
			try {
				std::ofstream ofs(yaml_path);
				ofs << root;
				ofs.close();
				RCLCPP_INFO(this->get_logger(), "Saved waypoints to %s", yaml_path.c_str());
				return root;
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to write yaml: %s", e.what());
				return YAML::Node();
			}
		}

		std::string WaypointEditor::duplicateMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			auto name = InteractiveMarkerManager::duplicateMarker(feedback);
			waypoints_.push_back(&controllable_markers_[name]);
			publishWaypointPath(waypoints_);
			return name;
		}

		void WaypointEditor::deleteMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::deleteMarker(feedback);

			waypoints_.erase(
				std::remove_if(waypoints_.begin(), waypoints_.end(), 
					[feedback](const auto &wp){ return wp->marker.name == feedback->marker_name; }),
				waypoints_.end());
			publishWaypointPath(waypoints_);
		}

		void WaypointEditor::changePosition(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::changePosition(feedback);
			
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				publishWaypointPath(waypoints_);
			}
		}

		void WaypointEditor::setInterpolate(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				std::string title;
				auto handle = feedback->menu_entry_id;
				if (menu_handler_->getTitle(handle, title))
				{
					interactive_markers::MenuHandler::CheckState state;
					if (menu_handler_->getCheckState(handle, state))
					{
						if (state == interactive_markers::MenuHandler::CheckState::UNCHECKED)
						{
							menu_handler_->setCheckState(handle, 
								interactive_markers::MenuHandler::CheckState::CHECKED);
							is_interpolate_ = true;
						}
						else if (state == interactive_markers::MenuHandler::CheckState::CHECKED)
						{
							menu_handler_->setCheckState(handle, 
								interactive_markers::MenuHandler::CheckState::UNCHECKED);
							is_interpolate_ = false;
						}
						menu_handler_->reApply(*imsrv_);
						imsrv_->applyChanges();
						publishWaypointPath(waypoints_);
					}
				}
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
			RCLCPP_DEBUG(this->get_logger(), "to_interpolate: %d, no_interpolate: %d", int(to_interpolate.size()), int(no_interpolate.size()));
		
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

		void WaypointEditor::publishWaypointPath(const std::vector<VisualMarker*> &waypoints)
		{
			std::vector<potbot_lib::Pose> waypoints_poses;
			for (const auto &wp:waypoints)
				waypoints_poses.push_back(potbot_lib::utility::get_pose(wp->marker.pose));

			if (is_interpolate_)
				waypoints_poses = interpolatePath(waypoints_poses);

			nav_msgs::msg::Path msg;
			potbot_lib::utility::to_msg(waypoints_poses, msg);
			msg.header.frame_id = frame_id_global_;
			msg.header.stamp = this->get_clock()->now();
			pub_waypoints_->publish(msg);
		}
	}
}