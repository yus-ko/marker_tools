#include <marker_tools/tree_map.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		TreeMap::TreeMap(std::string name, std::string node_namespace) : InteractiveMarkerManager(name,node_namespace)
		{
			fuction_duplicate_marker_ = 
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					auto name = this->duplicateMarker(feedback);
					addChild(feedback, name);
				};
			fuction_save_marker_ = [this](
				const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					auto yaml = this->saveMarker(feedback);
					saveTreeMap(feedback, yaml);
				};
			
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

			this->initialize();
			
			initializeMenu();
			initializeMarker(this->get_parameter("marker_yaml_path").as_string());
			
			pub_edges_ = this->create_publisher<visualization_msgs::msg::Marker>("edges", 1);

			// initializeMarker(this->get_parameter("marker_yaml_path").as_string());
			// initializeMarkerServer(controllable_markers_);
			
			RCLCPP_INFO(this->get_logger(), "TreeMap initialized");
			publishTreeMap(graph_);
		}

		// void TreeMap::initializeParameter()
		// {
		// 	this->declare_parameter("frame_id_global", rclcpp::ParameterValue("map"));
		// 	this->declare_parameter("marker_yaml_path", rclcpp::ParameterValue("interactive_markers.yaml"));
		// }

		void TreeMap::initializeMenu()
		{
			
		}

		void TreeMap::initializeMarker(std::string yaml_path)
		{
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

					RCLCPP_INFO(this->get_logger(), "Loaded: %s", yaml_path.c_str());
				}
			} 
			catch (const std::exception& e) 
			{
				RCLCPP_INFO(this->get_logger(), "Failed to load marker yaml: %s", e.what());

				auto ini_node_name = controllable_markers_.begin()->first;

				VisualMarkerGraphNode node;
				// node.id = 0;
				node.id = ini_node_name;
				node.data = &(controllable_markers_.begin()->second);
				graph_[ini_node_name] = node;

				RCLCPP_INFO(this->get_logger(), "Set root to %s", ini_node_name.c_str());
			}
		}

		rcl_interfaces::msg::SetParametersResult TreeMap::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
		{
			RCLCPP_INFO(this->get_logger(), "parameter changed");
			auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
			results->successful = true;

			return *results;
		}

		void TreeMap::markerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			// RCLCPP_INFO(this->get_logger(),"markerFeedback");
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				// breadthFirstSearch(id_start_node_, id_goal_node_);
				auto path = breadthFirstSearch("marker", "marker_3");
				publishTreeMap(graph_, path);
			}
		}

		std::vector<NodeId> TreeMap::breadthFirstSearch(const NodeId &start_id, const NodeId &goal_id)
		{
			std::vector<NodeId> checked_ids;
			std::queue<NodeId> tmp_queue;

			if (!utility::contains(start_id, graph_))
			{
				RCLCPP_WARN(this->get_logger(), "Please set start node");
				return checked_ids;
			}

			if (!utility::contains(goal_id, graph_))
			{
				RCLCPP_WARN(this->get_logger(), "Please set goal node");
				return checked_ids;
			}

			tmp_queue.push(start_id);

			while (!tmp_queue.empty())
			{
				NodeId current = tmp_queue.front(); tmp_queue.pop();
				checked_ids.push_back(current);
				if (current == goal_id)
					break;
				std::vector<NodeId> neighbors = graph_[current].children;
				for (const auto &neighbor : neighbors) 
				{
					if (!utility::contains(neighbor, checked_ids))
					{
						tmp_queue.push(neighbor);
					}
				}
			}

			// 経路復元
			std::vector<NodeId> path;
			path.push_back(goal_id);
			NodeId id = graph_[goal_id].parent;
			while (id != start_id)
			{
				path.push_back(id);
				id = graph_[id].parent;
			}
			path.push_back(start_id);
			std::reverse(path.begin(), path.end());

			return path;

		}

		void TreeMap::addChild(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, const std::string child_name)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				auto parent_name = int_marker.name;
				RCLCPP_DEBUG(this->get_logger(), "parent: %s, child: %s", parent_name.c_str(), child_name.c_str());

				auto &parent_node = graph_[parent_name];
				
				VisualMarkerGraphNode node;
				// node.id = graph_.size();
				node.id = child_name;
				node.parent = parent_node.id;
				node.data = &(controllable_markers_[child_name]);

				parent_node.children.push_back(node.id);
				graph_[child_name] = node;

				publishTreeMap(graph_);
			}
		}

		void TreeMap::saveTreeMap(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, YAML::Node &marker_yaml)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				for (int i = 0; i < marker_yaml["markers"].size(); i++) 
				{
					std::string name = marker_yaml["markers"][i]["name"].as<std::string>();

					auto graph_node = graph_[name];

					marker_yaml["markers"][i]["id"] = graph_node.id;
					marker_yaml["markers"][i]["parent"] = graph_node.parent;
					marker_yaml["markers"][i]["children"] = graph_node.children;
					// for (const auto id:graph_node.parent)
					// 	marker_yaml["markers"][i]["parent"].push_back(id);
					// for (const auto id:graph_node.children)
					// 	marker_yaml["markers"][i]["children"].push_back(id);
				}

				std::string yaml_path = this->get_parameter("marker_yaml_path").as_string();
				try {
					std::ofstream ofs(yaml_path);
					ofs << marker_yaml;
					ofs.close();
					RCLCPP_INFO(this->get_logger(), "Saved graph map to %s", yaml_path.c_str());
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to write yaml: %s", e.what());
				}
			}
		}

		void TreeMap::publishTreeMap(const std::map<NodeId, VisualMarkerGraphNode> &graph_map, const std::vector<NodeId> &path)
		{
			visualization_msgs::msg::Marker edges_msg;
			edges_msg.header.frame_id = frame_id_global_;
			edges_msg.header.stamp = this->get_clock()->now();
			edges_msg.id = 0;
			edges_msg.pose = utility::get_pose(0,0,0,0,0,0);
			edges_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
			edges_msg.scale.x = 0.01;
			// edges_msg.color = color::get_msg("green");

			for (const auto &node:graph_)
			{
				for (const auto id:node.second.children)
				{
					edges_msg.points.push_back(node.second.data->marker.pose.position);
					edges_msg.points.push_back(getNode(id).data->marker.pose.position);

					if (utility::contains(id, path))
					{
						edges_msg.colors.push_back(color::get_msg("blue"));
						edges_msg.colors.push_back(color::get_msg("blue"));
					}
					else
					{
						edges_msg.colors.push_back(color::get_msg("green"));
						edges_msg.colors.push_back(color::get_msg("green"));
					}
				}
			}

			pub_edges_->publish(edges_msg);
		}

		VisualMarkerGraphNode TreeMap::getNode(NodeId id)
		{
			for (const auto &node:graph_)
			{
				if (node.second.id == id)
				{
					return node.second;
				}
			}
			return VisualMarkerGraphNode();
		}
	}
}