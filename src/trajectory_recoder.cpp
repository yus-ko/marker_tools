#include <marker_tools/trajectory_recoder.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		TrajectoryRecoder::TrajectoryRecoder(std::string name, std::string node_namespace)
			: InteractiveMarkerManager(name,node_namespace)
		{	
		}

		void TrajectoryRecoder::initializeMenu()
		{
			InteractiveMarkerManager::initializeMenu();

			menu_handler_->insert(entry_handles_["delete"], "trajectory" , 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					clearTrajectory(feedback);});
			
			interactive_markers::MenuHandler::EntryHandle save_entry = menu_handler_->insert(
				entry_handles_["save"], "trajectory");

			menu_handler_->insert(save_entry, "csv", 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					saveTrajectory(feedback, "csv");});

			menu_handler_->insert(save_entry, "yaml", 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					saveTrajectory(feedback, "yaml");});
		}

		void TrajectoryRecoder::initializeMarker(std::string yaml_path, bool set_default)
		{
			InteractiveMarkerManager::initializeMarker(yaml_path, set_default);

			trajectory_.clear();
			auto now = this->get_clock()->now();
			for (const auto &cm:controllable_markers_)
			{
				auto marker_name = cm.second.marker.name;
				TrajectoryInfo info;
				std_msgs::msg::Header header;
				header.frame_id = frame_id_global_;
				header.stamp = now;
				info.headers.push_back(header);
				info.trajectory.push_back(
					potbot_lib::utility::get_pose(cm.second.marker.pose));

				info.publisher = this->create_publisher<nav_msgs::msg::Path>(
					marker_name + "/trajectory", 1);
				
				trajectory_[marker_name] = info;
			}
			
		}

		void TrajectoryRecoder::changePosition(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::changePosition(feedback);
			
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker) &&
				utility::contains(feedback->marker_name, trajectory_))
			{
				potbot_lib::Pose pose = potbot_lib::utility::get_pose(int_marker.pose);

				if (pose != trajectory_[int_marker.name].trajectory.back())
				// ((pose.to_affine().translation() - trajectory_[int_marker.name].trajectory.back().to_affine().translation()).norm() > 0.02)
				{
					std_msgs::msg::Header header;
					header.frame_id = frame_id_global_;
					header.stamp = this->get_clock()->now();
					trajectory_[int_marker.name].headers.push_back(header);
					trajectory_[int_marker.name].trajectory.push_back(pose);

					// trajectory_[int_marker.name].trajectory = interpolateTrajectory(trajectory_[int_marker.name].trajectory);
					
					trajectory_[int_marker.name].publish();
				}				

				RCLCPP_DEBUG(this->get_logger(),"[%s]: pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, size: %d",
					int_marker.name.c_str(),
					pose.position.x, pose.position.y, pose.position.z,
					pose.rotation.x, pose.rotation.y, pose.rotation.z,
					int(trajectory_[int_marker.name].trajectory.size()));
			}
		}

		void TrajectoryRecoder::clearTrajectory(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker) &&
				utility::contains(feedback->marker_name, trajectory_))
			{
				auto latest_pose = trajectory_[int_marker.name].trajectory.back();
				auto latest_header = trajectory_[int_marker.name].headers.back();
				trajectory_[int_marker.name].trajectory.clear();
				trajectory_[int_marker.name].headers.clear();
				trajectory_[int_marker.name].trajectory.push_back(latest_pose);
				trajectory_[int_marker.name].headers.push_back(latest_header);
				trajectory_[int_marker.name].publish();
				RCLCPP_INFO(this->get_logger(),"[%s]: trajectory cleared", int_marker.name.c_str());
			}
		}

		void TrajectoryRecoder::saveTrajectory(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string type)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker) &&
				utility::contains(feedback->marker_name, trajectory_))
			{
				std::string file_path = int_marker.name + "_trajectory." + type;

				bool successful = true;
				if (type == "csv")
				{
					successful = saveCSV(int_marker.name, file_path);
				}
				else if (type == "yaml")
				{
					successful = saveYAML(int_marker.name, file_path);
				}

				if (successful)
				{
					RCLCPP_INFO(this->get_logger(),"[%s]: trajectory saved", int_marker.name.c_str());
					RCLCPP_INFO(this->get_logger(),"[%s]: %s", int_marker.name.c_str(), file_path.c_str());
				}
				else
				{
					RCLCPP_WARN(this->get_logger(),"[%s]: trajectory save failed", int_marker.name.c_str());
					RCLCPP_WARN(this->get_logger(),"[%s]: %s", int_marker.name.c_str(), file_path.c_str());
				}
			}
		}

		std::vector<potbot_lib::Pose> TrajectoryRecoder::interpolateTrajectory(const std::vector<potbot_lib::Pose>& trajectory)
		{	
			auto points_num = trajectory.size();
			if (points_num < 2)
			{
				return trajectory;
			}

			size_t interpolate_num_limit = 50;
			static size_t last_interpolated_index = 0;
			std::vector<Pose> no_interpolate, to_interpolate, interpolated, out;
			if (points_num > interpolate_num_limit)
			{
				to_interpolate = std::vector<Pose>(trajectory.end() - interpolate_num_limit, trajectory.end());
				no_interpolate = std::vector<Pose>(trajectory.begin(), trajectory.end() - interpolate_num_limit);
			}
			else
				to_interpolate = trajectory;

			interpolate::bezier(to_interpolate, to_interpolate.size(), interpolated);
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

		// bool directoryExists(const std::string &path) 
		// {
		// 	struct stat info;
		// 	if (stat(path.c_str(), &info) != 0) 
		// 	{
		// 		return false; // ディレクトリは存在しない
		// 	}
		// 	else if (info.st_mode & S_IFDIR) 
		// 	{
		// 		return true; // ディレクトリは存在する
		// 	} 
		// 	else 
		// 	{
		// 		return false; // パスはディレクトリではない
		// 	}
		// }

		// bool createDirectory(const std::string &path) 
		// {
		// 	if (mkdir(path.c_str(), 0755) != 0) 
		// 	{
		// 		if (errno == EEXIST) 
		// 		{
		// 			return true; // ディレクトリは既に存在する
		// 		} else 
		// 		{
		// 			return false; // ディレクトリの作成に失敗
		// 		}
		// 	}
		// 	return true; // ディレクトリの作成に成功
		// }

		// bool createDirectoriesRecursively(const std::string &path) 
		// {
		// 	size_t pos = 0;
		// 	std::string current_path;

		// 	while ((pos = path.find_first_of('/', pos)) != std::string::npos) 
		// 	{
		// 		current_path = path.substr(0, pos++);
		// 		if (!current_path.empty() && !directoryExists(current_path)) 
		// 		{
		// 			if (!createDirectory(current_path)) 
		// 			{
		// 				return false;
		// 			}
		// 		}
		// 	}

		// 	if (!directoryExists(path)) {
		// 		if (!createDirectory(path)) 
		// 		{
		// 			return false;
		// 		}
		// 	}

		// 	return true;
		// }

		bool TrajectoryRecoder::saveCSV(std::string marker_name, std::string file_path)
		{

			// size_t last_slash_pos = csv_path.find_last_of('/');
			// std::string directory_path = csv_path.substr(0, last_slash_pos);

			// ディレクトリが存在するか確認し、存在しない場合は作成
			// if (!directoryExists(directory_path)) 
			// {
			// 	if (!createDirectoriesRecursively(directory_path)) 
			// 	{
			// 		ROS_ERROR_STREAM("Failed to create directory: " << directory_path);
			// 		resp.success = false;
			// 		resp.message = "Failed to create directory: " + directory_path;
			// 		return false;
			// 	}
			// }

			std::ofstream csv_file(file_path);

			if (csv_file.is_open()) 
			{
				for (const auto& p : trajectory_[marker_name].trajectory) 
				{
					csv_file	<< p.position.x << ","
								<< p.position.y << ","
								<< p.position.z << ","
								<< p.rotation.x << ","
								<< p.rotation.y << ","
								<< p.rotation.z << "\n";
				}
				csv_file.close();
				return true;
			}
			else
			{
				return false;
			}
		}

		bool TrajectoryRecoder::saveYAML(std::string marker_name, std::string file_path)
		{
			YAML::Node root;
			YAML::Node node = root[marker_name]["path"];

			for (const auto& p:trajectory_[marker_name].trajectory)
			{
				YAML::Node child;

				child["pose"]["position"]["x"] = p.position.x;
				child["pose"]["position"]["y"] = p.position.y;
				child["pose"]["position"]["z"] = p.position.z;

				child["pose"]["rotation"]["roll"] = p.rotation.x;
				child["pose"]["rotation"]["pitch"] = p.rotation.y;
				child["pose"]["rotation"]["yaw"] = p.rotation.z;

				node.push_back(child);
			}

			try 
			{
				std::ofstream ofs(file_path);
				ofs << root;
				ofs.close();
				return true;
			} 
			catch (const std::exception& e) 
			{
				return false;
			}
		}

		// std::vector<VisualMarker>* TrajectoryRecoder::getVisualMarker()
		// {
		// 	return &visual_markers_;
		// }
	}
}