#include <marker_tools/tree_map.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		TreeMap::TreeMap(std::string name, std::string node_namespace) : InteractiveMarkerManager(name,node_namespace)
		{
			pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("edges", 1);
		}

		// void TreeMap::initializeParameter()
		// {
		// 	this->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		// 	this->declare_parameter("marker_yaml_path", rclcpp::ParameterValue("interactive_markers.yaml"));
		// }

		void TreeMap::initializeMenu()
		{
			InteractiveMarkerManager::initializeMenu();

			auto set_handle = menu_handler_->insert("set");

			menu_handler_->insert(set_handle, "start" , 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					setStartNode(feedback);});

			menu_handler_->insert(set_handle, "goal" , 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					setGoalNode(feedback);});

		}

		void TreeMap::initializeMarker(std::string yaml_path, bool set_default)
		{
			InteractiveMarkerManager::initializeMarker(yaml_path,set_default);
			try 
			{
				YAML::Node root = YAML::LoadFile(yaml_path);
				if (root["markers"]) 
				{
					for (const auto& node : root["markers"]) 
					{
						std::string name = node["name"].as<std::string>();
						graph_[name].id = node["id"].as<NodeId>();
						graph_[name].parent = node["parent"].as<NodeId>();
						graph_[name].children = node["children"].as<std::vector<NodeId>>();
						graph_[name].data = &(controllable_markers_[name]);
					}

					RCLCPP_INFO(this->get_logger(), "Tree map loaded: %s", yaml_path.c_str());
				}
			} 
			catch (const std::exception& e) 
			{
				RCLCPP_INFO(this->get_logger(), "Failed to tree map load marker yaml: %s", e.what());

				auto ini_node_name = controllable_markers_.begin()->first;

				VisualMarkerTreeNode node;
				// node.id = 0;
				node.id = ini_node_name;
				node.data = &(controllable_markers_.begin()->second);
				graph_[ini_node_name] = node;

				RCLCPP_INFO(this->get_logger(), "Set root to %s", ini_node_name.c_str());
			}
		}

		void TreeMap::initializeMarkerServer(const std::map<std::string, VisualMarker> &markers)
		{
			InteractiveMarkerManager::initializeMarkerServer(markers);
			publishTreeMap(graph_);
		}

		rcl_interfaces::msg::SetParametersResult TreeMap::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
		{
			RCLCPP_INFO(this->get_logger(), "parameter changed");
			auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
			results->successful = true;

			return *results;
		}

		void TreeMap::changePosition(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::changePosition(feedback);
			
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				if (utility::contains(id_start_node_, graph_) &&
					utility::contains(id_goal_node_, graph_) &&
					id_start_node_ != id_goal_node_)
				{
					auto path = breadthFirstSearch(id_start_node_, id_goal_node_);
					publishTreeMap(graph_, path);
				}
				else
				{
					publishTreeMap(graph_);
				}
			}
		}

		void TreeMap::setStartNode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				id_start_node_ = graph_[int_marker.name].id;
				// for (auto &node:graph_)
				// 	if (node.first != id_goal_node_)
				// 		node.second.data->marker.controls.front().markers.front().color
				// 			= color::get_msg("red");
				// graph_[id_start_node_].data->marker.controls.front().markers.front().color
				// 	 = color::get_msg("green");
				// initializeMarkerServer(controllable_markers_);
				RCLCPP_INFO(this->get_logger(), "Set start node: %s", int_marker.name.c_str());
			}
		}

		void TreeMap::setGoalNode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				id_goal_node_ = graph_[int_marker.name].id;
				// for (auto &node:graph_)
				// 	if (node.first != id_start_node_)
				// 		node.second.data->marker.controls.front().markers.front().color
				// 			= color::get_msg("red");
				// graph_[id_goal_node_].data->marker.controls.front().markers.front().color
				// 	 = color::get_msg("blue");
				initializeMarkerServer(controllable_markers_);
				RCLCPP_INFO(this->get_logger(), "Set goal node: %s", int_marker.name.c_str());
			}
		}

		std::vector<NodeId> TreeMap::breadthFirstSearch(const NodeId &start_id, const NodeId &goal_id)
		{
			std::unordered_map<NodeId, NodeId> checked_ids;
			std::vector<NodeId> path;
			std::queue<NodeId> tmp_queue;

			if (!utility::contains(start_id, graph_))
			{
				RCLCPP_WARN(this->get_logger(), "Please set start node");
				return path;
			}

			if (!utility::contains(goal_id, graph_))
			{
				RCLCPP_WARN(this->get_logger(), "Please set goal node");
				return path;
			}

			tmp_queue.push(start_id);

			while (!tmp_queue.empty())
			{
				NodeId current = tmp_queue.front(); tmp_queue.pop();

				if (current == goal_id)
					break;

				std::vector<NodeId> neighbors = graph_[current].children;
				neighbors.push_back(graph_[current].parent);
				for (const auto &neighbor : neighbors) 
				{
					if (!utility::contains(neighbor, checked_ids))
					{
						tmp_queue.push(neighbor);
						checked_ids[neighbor] = current;
					}
				}
			}

			// 経路復元
			for (NodeId n = goal_id; n != start_id; n = checked_ids[n])
			{
				path.push_back(n);
			}
			path.push_back(start_id);
			std::reverse(path.begin(), path.end());
			
			return path;

		}

		std::string TreeMap::duplicateMarker(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			auto child_name = InteractiveMarkerManager::duplicateMarker(feedback);

			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				auto parent_name = int_marker.name;
				RCLCPP_DEBUG(this->get_logger(), "parent: %s, child: %s", parent_name.c_str(), child_name.c_str());

				auto &parent_node = graph_[parent_name];
				
				VisualMarkerTreeNode node;
				// node.id = graph_.size();
				node.id = child_name;
				node.parent = parent_node.id;
				node.data = &(controllable_markers_[child_name]);

				parent_node.children.push_back(node.id);
				graph_[child_name] = node;

				publishTreeMap(graph_);
			}

			return child_name;
		}

		void TreeMap::deleteMarker(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::deleteMarker(feedback);

			auto node_name = feedback->marker_name;
			auto parent_name = graph_[node_name].parent;

			for (auto &child_name:graph_[node_name].children)
				graph_[child_name].parent = parent_name;

			auto it = std::find(	graph_[parent_name].children.begin(), 
									graph_[parent_name].children.end(),
									node_name);
			graph_[parent_name].children.erase(it);
			
			graph_[parent_name].children.insert(
				graph_[parent_name].children.end(),
				graph_[node_name].children.begin(),
				graph_[node_name].children.end());

			graph_.erase(node_name);

			publishTreeMap(graph_);	
		}

		YAML::Node TreeMap::saveMarker(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			auto base_yaml = InteractiveMarkerManager::saveMarker(feedback);

			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				for (int i = 0; i < base_yaml["markers"].size(); i++) 
				{
					std::string name = base_yaml["markers"][i]["name"].as<std::string>();

					auto graph_node = graph_[name];

					base_yaml["markers"][i]["id"] = graph_node.id;
					base_yaml["markers"][i]["parent"] = graph_node.parent;
					base_yaml["markers"][i]["children"] = graph_node.children;
					// for (const auto id:graph_node.parent)
					// 	marker_yaml["markers"][i]["parent"].push_back(id);
					// for (const auto id:graph_node.children)
					// 	marker_yaml["markers"][i]["children"].push_back(id);
				}

				std::string yaml_path = this->get_parameter("marker_yaml_path").as_string();
				try {
					std::ofstream ofs(yaml_path);
					ofs << base_yaml;
					ofs.close();
					RCLCPP_INFO(this->get_logger(), "Saved tree map to %s", yaml_path.c_str());
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to write yaml: %s", e.what());
				}
			}

			return base_yaml;

		}

		void TreeMap::publishTreeMap(
			const std::map<NodeId, VisualMarkerTreeNode> &graph_map, const std::vector<NodeId> &path)
		{
			visualization_msgs::msg::MarkerArray edges_msg_array;

			visualization_msgs::msg::Marker edges_msg;
			edges_msg.header.frame_id = frame_id_global_;
			edges_msg.header.stamp = this->get_clock()->now();
			edges_msg.id = 0;
			edges_msg.ns = "edge";
			edges_msg.pose = utility::get_pose(0,0,0,0,0,0);
			edges_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
			edges_msg.scale.x = 0.01;
			edges_msg.color = color::get_msg("green");

			for (const auto &node:graph_)
			{
				for (const auto id:node.second.children)
				{
					edges_msg.points.push_back(node.second.data->marker.pose.position);
					edges_msg.points.push_back(getNode(id).data->marker.pose.position);
				}
			}
			edges_msg_array.markers.push_back(edges_msg);

			visualization_msgs::msg::Marker path_msg = edges_msg;
			path_msg.id = 1;
			path_msg.ns = "path";
			path_msg.action = visualization_msgs::msg::Marker::DELETE;
			path_msg.points.clear();
			if (!path.empty())
			{
				path_msg.action = visualization_msgs::msg::Marker::MODIFY;
				path_msg.color = color::get_msg("blue");
				// path_msg.scale.x = 0.02;
				for (int i = 0; i < path.size()-1; i++)
				{
					path_msg.points.push_back(graph_[path[i]].data->marker.pose.position);
					path_msg.points.push_back(graph_[path[i+1]].data->marker.pose.position);
				}
			}
			edges_msg_array.markers.push_back(path_msg);

			pub_edges_->publish(edges_msg_array);
		}

		VisualMarkerTreeNode TreeMap::getNode(NodeId id)
		{
			for (const auto &node:graph_)
			{
				if (node.second.id == id)
				{
					return node.second;
				}
			}
			return VisualMarkerTreeNode();
		}
	}
}