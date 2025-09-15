#include <marker_tools/trajectory_recoder.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		TrajectoryRecoder::TrajectoryRecoder(std::string name, std::string node_namespace) : InteractiveMarkerManager(name,node_namespace)
		{
			initializeMenu();
			initializeMarker();
			
			for (auto &tr:trajectory_)
			{
				tr.second.publisher = this->create_publisher<nav_msgs::msg::Path>(
					tr.first + "/trajectory", 1);
			}

			// srv_save_marker_trajectory_ = pnh.advertiseService("save_marker_tarajectory", &InteractiveMarkerManager::serviceSaveMarkerTrajectory, this);
			// srv_clear_marker_trajectory_ = pnh.advertiseService("clear_marker_tarajectory", &InteractiveMarkerManager::serviceClearMarkerTrajectory, this);
			
			RCLCPP_INFO(this->get_logger(), "TrajectoryRecoder initialized");
		}

		// void TrajectoryRecoder::initializeParameter()
		// {
		// 	this->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		// 	this->declare_parameter("marker_yaml_path", rclcpp::ParameterValue("interactive_markers.yaml"));
		// }

		void TrajectoryRecoder::initializeMenu()
		{
			interactive_markers::MenuHandler::EntryHandle traj_entry = menu_handler_->insert("trajectory");

			menu_handler_->insert(traj_entry, "clear" , 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->clearTrajectory(feedback);});
			
			interactive_markers::MenuHandler::EntryHandle save_entry = menu_handler_->insert(
				entry_handle_save_, "trajectory");

			menu_handler_->insert(save_entry, "csv", 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->saveTrajectory(feedback, "csv");});

			menu_handler_->insert(save_entry, "yaml", 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->saveTrajectory(feedback, "yaml");});
			
			for (const auto &cm:controllable_markers_)
			{
				menu_handler_->apply(*imsrv_, cm.first);
			}
			imsrv_->applyChanges();
		}

		void TrajectoryRecoder::initializeMarker()
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
			
			trajectory_.clear();
			for (const auto &cm:controllable_markers_)
			{
				auto marker_name = cm.first;
				registerFeedback(marker_name, function_change_position_);
				TrajectoryInfo info;
				std_msgs::msg::Header header;
				header.frame_id = frame_id_global_;
				header.stamp = this->get_clock()->now();
				info.headers.push_back(header);
				info.trajectory.push_back(
					potbot_lib::utility::get_pose(cm.second.marker.pose));
				
				trajectory_[marker_name] = info;
			}
			
		}

		rcl_interfaces::msg::SetParametersResult TrajectoryRecoder::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
		{
			RCLCPP_INFO(this->get_logger(), "パラメータ変更");
			auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
			results->successful = true;

			// int id = param.marker_id;
			// std::string visual_type = param.trajectory_marker_type;

			// u_int8_t type = visualization_msgs::Marker::LINE_STRIP;
			// if (visual_type == "line")
			// {
			// 	type = visualization_msgs::Marker::LINE_STRIP;
			// }
			// else if (visual_type == "points")
			// {
			// 	type = visualization_msgs::Marker::POINTS;
			// }

			// std::vector<size_t> ids;
			// if (id == -1)
			// {
			// 	for (size_t i = 0; i < visual_markers_.size(); i++)
			// 	{
			// 		ids.push_back(id);
			// 	}
			// }
			// else
			// {
			// 	ids.push_back(id);
			// }
			
			// for (const auto& i:ids)
			// {
			// 	if (i < visual_markers_.size())
			// 	{
			// 		visual_markers_[i].trajectory_recording = param.trajectory_recording;
			// 		if (visual_markers_[i].trajectory_recording)
			// 		{
			// 			visual_markers_[i].trajectory_marker_type = type;
			// 			visual_markers_[i].trajectory_interpolation_method = param.trajectory_interpolation_method;
			// 		}
			// 		else
			// 		{
			// 			visual_markers_[i].trajectory.clear();
			// 		}
			// 	}
			// }
			// publishMarkerTrajectory();
			
			// initializeMarker();

			return *results;
		}

		// void InteractiveMarkerManager::reconfigureCB(const potbot_lib::MarkerManagerConfig& param, uint32_t level)
		// {	
		// 	static bool first = true;

		// 	if (!first)
		// 	{
		// 		int id = param.marker_id;
		// 		std::string visual_type = param.trajectory_marker_type;

		// 		u_int8_t type = visualization_msgs::Marker::LINE_STRIP;
		// 		if (visual_type == "line")
		// 		{
		// 			type = visualization_msgs::Marker::LINE_STRIP;
		// 		}
		// 		else if (visual_type == "points")
		// 		{
		// 			type = visualization_msgs::Marker::POINTS;
		// 		}

		// 		std::vector<size_t> ids;
		// 		if (id == -1)
		// 		{
		// 			for (size_t i = 0; i < visual_markers_.size(); i++)
		// 			{
		// 				ids.push_back(id);
		// 			}
		// 		}
		// 		else
		// 		{
		// 			ids.push_back(id);
		// 		}
				
		// 		for (const auto& i:ids)
		// 		{
		// 			if (i < visual_markers_.size())
		// 			{
		// 				visual_markers_[i].trajectory_recording = param.trajectory_recording;
		// 				if (visual_markers_[i].trajectory_recording)
		// 				{
		// 					visual_markers_[i].trajectory_marker_type = type;
		// 					visual_markers_[i].trajectory_interpolation_method = param.trajectory_interpolation_method;
		// 				}
		// 				else
		// 				{
		// 					visual_markers_[i].trajectory.clear();
		// 				}
		// 			}
		// 		}
		// 		publishMarkerTrajectory();
		// 	}

		// 	first = false;
		// }

		void TrajectoryRecoder::markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			// RCLCPP_INFO(this->get_logger(),"markerFeedback");
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker) && contains(feedback->marker_name, trajectory_))
			{
				potbot_lib::Pose pose = potbot_lib::utility::get_pose(int_marker.pose);

				if (pose != trajectory_[int_marker.name].trajectory.back())
				{
					std_msgs::msg::Header header;
					header.frame_id = frame_id_global_;
					header.stamp = this->get_clock()->now();
					trajectory_[int_marker.name].headers.push_back(header);
					trajectory_[int_marker.name].trajectory.push_back(pose);
					
					trajectory_[int_marker.name].publish();
				}				

				RCLCPP_DEBUG(this->get_logger(),"[%s]: pose: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, size: %d",
					int_marker.name.c_str(),
					pose.position.x, pose.position.y, pose.position.z,
					pose.rotation.x, pose.rotation.y, pose.rotation.z,
					int(trajectory_[int_marker.name].trajectory.size()));
			}
		}

		void TrajectoryRecoder::clearTrajectory(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker) && contains(feedback->marker_name, trajectory_))
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
			if (imsrv_->get(feedback->marker_name, int_marker) && contains(feedback->marker_name, trajectory_))
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

		void TrajectoryRecoder::interpolateTrajectory(size_t id)
		{
			// if (visual_markers_[id].trajectory.empty())
			// {
			// 	geometry_msgs::msg::PoseStamped p;
			// 	p.header = visual_markers_[id].marker.header;
			// 	p.pose = visual_markers_[id].marker.pose;
			// 	visual_markers_[id].trajectory.push_back(p);
			// }
			
			// double distance_to_pre = utility::get_distance(visual_markers_[id].marker.pose, visual_markers_[id].trajectory.back().pose);
			// if (distance_to_pre > 0.01)
			// {
			// 	geometry_msgs::msg::PoseStamped pose;
			// 	pose.header.frame_id = frame_id_global_;
			// 	pose.header.stamp = this->get_clock()->now();
			// 	pose.pose = visual_markers_[id].marker.pose;

			// 	if (distance_to_pre > 0.05)
			// 	{
			// 		std::vector<geometry_msgs::msg::PoseStamped> start_end(2);
			// 		start_end[0] = visual_markers_[id].trajectory.back();
			// 		start_end[1] = pose;
					
			// 		std::vector<Eigen::Vector2d> interp_vecs;
			// 		utility::to_mat(start_end,interp_vecs);
			// 		interpolate::linear(interp_vecs, int(distance_to_pre/0.05), interp_vecs);
			// 		utility::to_msg(interp_vecs,start_end);

			// 		RCLCPP_DEBUG(this->get_logger(), "distance_to_pre: %f, linear_interpolate_num: %d", distance_to_pre, (int)start_end.size());

			// 		for (size_t i = 1; i < start_end.size(); i++)
			// 		{
			// 			start_end[i].header = pose.header;
			// 			visual_markers_[id].trajectory.push_back(start_end[i]);
			// 		}
			// 	}
			// 	else
			// 	{
			// 		visual_markers_[id].trajectory.push_back(pose);
			// 	}

			// 	auto* points = &visual_markers_[id].trajectory;
			// 	size_t num = points->size();
			// 	size_t interpolate_num_limit = 50;
			// 	static size_t last_interpolated_index = 0;
			// 	if (num > 1)
			// 	{
			// 		std::vector<geometry_msgs::msg::PoseStamped> split_traj;
			// 		if (num > interpolate_num_limit)
			// 		{
			// 			for (size_t i = num - interpolate_num_limit; i < num; i++)
			// 			{
			// 				split_traj.push_back((*points)[i]);
			// 			}
			// 		}
			// 		else
			// 		{
			// 			split_traj = *points;
			// 		}
					
			// 		if (split_traj.size() > 1)
			// 		{
			// 			std::vector<Eigen::Vector2d> traj_vecs;
			// 			utility::to_mat(split_traj,traj_vecs);
			// 			if (visual_markers_[id].trajectory_interpolation_method == "spline")
			// 			{
			// 				interpolate::spline(traj_vecs, traj_vecs.size(), traj_vecs);
			// 			}
			// 			else if (visual_markers_[id].trajectory_interpolation_method == "bezier")
			// 			{
			// 				interpolate::bezier(traj_vecs, traj_vecs.size(), traj_vecs);
			// 			}
			// 			utility::to_msg(traj_vecs,split_traj);
			// 		}

			// 		if (num > interpolate_num_limit)
			// 		{
			// 			for (size_t i = 0; i < split_traj.size(); i++)
			// 			{
			// 				visual_markers_[id].trajectory[i+num-interpolate_num_limit] = split_traj[i];
			// 			}
			// 		}
			// 		else
			// 		{
			// 			visual_markers_[id].trajectory = split_traj;
			// 		}
			// 		RCLCPP_DEBUG(this->get_logger(), "interpolate size: %d, trajectories_[%d]_size: %d", (int)split_traj.size(), (int)id, (int)visual_markers_[id].trajectory.size());

			// 	}
			// 	publishMarkerTrajectory();
			// }
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

		bool TrajectoryRecoder::contains(std::string key, const std::map<std::string, TrajectoryInfo> &map)
		{
			auto it = map.find(key);
			return (it != map.end());
		}

		// bool InteractiveMarkerManager::serviceClearMarkerTrajectory(potbot_lib::Save::Request &req, potbot_lib::Save::Response &resp)
		// {
		// 	std::string marker_name = req.save_target;

		// 	if (marker_name == "")
		// 	{
		// 		std::string names = "";
		// 		for (auto& vm:visual_markers_)
		// 		{
		// 			vm.trajectory.clear();
		// 			names+=vm.marker.text + ", ";
		// 		}
		// 		names.pop_back();
		// 		names.pop_back();
		// 		ROS_INFO_STREAM("Clear trajectory: " << names);
		// 		resp.success = true;
		// 		resp.message = "Clear trajectory: " + names;
		// 	}
		// 	else
		// 	{
		// 		int id = getMarkerId(marker_name);
		// 		if (id == -1)
		// 		{
		// 			ROS_INFO_STREAM("Invalid target name: " << marker_name);
		// 			resp.success = false;
		// 			resp.message = "Invalid target name: " + marker_name;
		// 			return false;
		// 		}
		// 		else
		// 		{
		// 			visual_markers_[id].trajectory.clear();
		// 			ROS_INFO_STREAM("Clear trajectory: " << visual_markers_[id].marker.text);
		// 			resp.success = true;
		// 			resp.message = "Clear trajectory: " + visual_markers_[id].marker.text;
		// 		}
		// 	}
		// 	return true;
		// }

		// std::vector<VisualMarker>* TrajectoryRecoder::getVisualMarker()
		// {
		// 	return &visual_markers_;
		// }
	}
}