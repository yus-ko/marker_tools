#include <marker_tools/graph_map.hpp>

using namespace std::chrono_literals;

namespace potbot_lib{
	namespace marker_tools{

		GraphMap::GraphMap(std::string name, std::string node_namespace) : TreeMap(name,node_namespace)
		{
		}

		void GraphMap::initializeMenu()
		{
			TreeMap::initializeMenu();

			menu_handler_->insert(entry_handles_["add"], "edge",
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					changeToConnectMode(feedback);});
			
			menu_handler_connect_ = std::make_shared<interactive_markers::MenuHandler>();
			menu_handler_connect_->insert("connect",
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					this->connectNode(connection_source_, feedback->marker_name);});

			menu_handler_cancel_ = std::make_shared<interactive_markers::MenuHandler>();
			menu_handler_cancel_->insert("cancel",
				[this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
					// this->changeToNormalMode(feedback);
					initializeMarkerServer(controllable_markers_);});

		}

		void GraphMap::changeToConnectMode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				for (const auto &cm:controllable_markers_)
				{
					if (cm.first == int_marker.name)
					{
						menu_handler_cancel_->apply(*imsrv_, cm.first);
					}
					else
					{
						menu_handler_connect_->apply(*imsrv_, cm.first);
					}
				}
				imsrv_->applyChanges();
				connection_source_ = graph_[int_marker.name].id;
			}
		}

		void GraphMap::changeToNormalMode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				initializeMarkerServer(controllable_markers_);
			}
		}

		void GraphMap::connectNode(const NodeId &source, const NodeId &target)
		{
			if (utility::contains(source, graph_) && utility::contains(target, graph_))
			{
				graph_[source].connections.insert(target);
				graph_[target].connections.insert(source);
				RCLCPP_INFO(this->get_logger(), "Node conected: %s <-> %s",
					graph_[source].id.c_str(), graph_[target].id.c_str());
				initializeMarkerServer(controllable_markers_);
			}
		}

		void GraphMap::initializeMarker(std::string yaml_path, bool set_default)
		{
			InteractiveMarkerManager::initializeMarker(yaml_path, set_default);

			try 
			{
				YAML::Node root = YAML::LoadFile(yaml_path);
				if (root["markers"]) 
				{
					for (const auto& node : root["markers"]) 
					{
						std::string name = node["name"].as<std::string>();
						graph_[name].id = node["id"].as<NodeId>();
						auto vec = node["connections"].as<std::vector<NodeId>>();
						graph_[name].connections = std::set(vec.begin(), vec.end());
						graph_[name].data = &(controllable_markers_[name]);
					}

					RCLCPP_INFO(this->get_logger(), "Graph map loaded: %s", yaml_path.c_str());
				}
			} 
			catch (const std::exception& e) 
			{
				RCLCPP_INFO(this->get_logger(), "Failed to graph map load marker yaml: %s", e.what());

				auto ini_node_name = controllable_markers_.begin()->first;

				VisualMarkerGraphNode node;
				// node.id = 0;
				node.id = ini_node_name;
				node.data = &(controllable_markers_.begin()->second);
				graph_[ini_node_name] = node;

				RCLCPP_INFO(this->get_logger(), "Set root to %s", ini_node_name.c_str());
			}
		}

		void GraphMap::initializeMarkerServer(
			const std::map<std::string, VisualMarker> &markers, const std::vector<std::string> &marker_with_controller)
		{
			InteractiveMarkerManager::initializeMarkerServer(markers, marker_with_controller);
			publishGraphMap(graph_);
		}

		void GraphMap::changePosition(
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
					publishGraphMap(graph_, getPath(id_start_node_, id_goal_node_));
				}
				else
				{
					publishGraphMap(graph_);
				}
			}
		}

		void GraphMap::setStartNode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				id_start_node_ = graph_[int_marker.name].id;
				RCLCPP_INFO(this->get_logger(), "Set start node: %s", id_start_node_.c_str());
				changePosition(feedback);
			}
		}

		void GraphMap::setGoalNode(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				id_goal_node_ = graph_[int_marker.name].id;
				RCLCPP_INFO(this->get_logger(), "Set goal node: %s", id_goal_node_.c_str());
				changePosition(feedback);
			}
		}

		YAML::Node GraphMap::saveMarker(
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
					base_yaml["markers"][i]["connections"] = std::vector(graph_node.connections.begin(), graph_node.connections.end());
				}

				std::string yaml_path = marker_file_;
				try {
					std::ofstream ofs(yaml_path);
					ofs << base_yaml;
					ofs.close();
					RCLCPP_INFO(this->get_logger(), "Saved graph map to %s", yaml_path.c_str());
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Failed to write yaml: %s", e.what());
				}
			}

			return base_yaml;
		}

		std::string GraphMap::duplicateMarker(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			auto child_name = InteractiveMarkerManager::duplicateMarker(feedback);

			visualization_msgs::msg::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker))
			{
				auto parent_name = int_marker.name;

				auto &parent_node = graph_[parent_name];
				
				VisualMarkerGraphNode node;
				node.id = child_name;
				node.data = &(controllable_markers_[child_name]);
				node.connections.insert(parent_name);
				parent_node.connections.insert(node.id);
				
				graph_[child_name] = node;

				publishGraphMap(graph_);
			}

			return child_name;
		}

		void GraphMap::deleteMarker(
			const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
		{
			InteractiveMarkerManager::deleteMarker(feedback);

			auto node_name = feedback->marker_name;

			for (const auto &node:graph_[node_name].connections)
			{
				auto it = std::find(graph_[node].connections.begin(), 
									graph_[node].connections.end(),
									node_name);
				graph_[node].connections.erase(it);
			}	

			graph_.erase(node_name);

			publishGraphMap(graph_);
		}

		std::vector<NodeId> GraphMap::getPath(const NodeId &start, const NodeId &goal)
		{

			std::vector<NodeId> path;
			if (!utility::contains(start, graph_)) {
				RCLCPP_WARN(this->get_logger(), "Please set start node");
				return path;
			}
			if (!utility::contains(goal, graph_)) {
				RCLCPP_WARN(this->get_logger(), "Please set goal node");
				return path;
			}

			std::map<NodeId, double> cost;
			std::map<NodeId, NodeId> parent;
			std::map<NodeId, bool> visited;
			for (const auto &g : graph_) 
			{
				cost[g.second.id] = std::numeric_limits<double>::infinity();
				visited[g.second.id] = false;
			}
			cost[start] = 0;
			parent[start] = start;

			while (true) 
			{
				// 未訪問かつコスト最小のノードを選択
				NodeId current;
				double min_cost = std::numeric_limits<double>::infinity();
				bool found = false;
				for (const auto &kv : cost) 
				{
					if (!visited[kv.first] && kv.second < min_cost) 
					{
						min_cost = kv.second;
						current = kv.first;
						found = true;
					}
				}
				if (!found) 
					break; // 到達不能
				if (current == goal) 
					break; // ゴール到達
				visited[current] = true;

				auto &current_node = graph_[current];
				for (const auto &conn : current_node.connections) 
				{
					if (!visited[conn]) 
					{
						double dist = (utility::get_point(current_node.data->marker.pose.position) -
									   utility::get_point(graph_[conn].data->marker.pose.position)).norm();
						double new_cost = cost[current] + dist;
						if (new_cost < cost[conn]) 
						{
							cost[conn] = new_cost;
							parent[conn] = current;
						}
					}
				}
			}

			// 経路復元
			if (!parent.count(goal)) 
			{
				RCLCPP_WARN(this->get_logger(), "Path not found");
				return path;
			}
			NodeId current = goal;
			while (current != start) 
			{
				path.push_back(current);
				current = parent[current];
			}
			path.push_back(start);
			std::reverse(path.begin(), path.end());
			return path;
		}

		void GraphMap::publishGraphMap(const std::map<NodeId, VisualMarkerGraphNode> &graph_map, const std::vector<NodeId> &path)
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
				for (const auto id:node.second.connections)
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

			// visualization_msgs::msg::Marker mesh_msg;
			// mesh_msg.header = edges_msg.header;
			// mesh_msg.id = 2;
			// mesh_msg.ns = "mesh";

			// mesh_msg.scale.x = 1;
			// mesh_msg.scale.y = 1;
			// mesh_msg.scale.z = 1;
			// // int_marker.controls.front().markers.front().color.r = 0;
			// // int_marker.controls.front().markers.front().color.g = 0;
			// // int_marker.controls.front().markers.front().color.b = 0;
			// // int_marker.controls.front().markers.front().color.a = 0;
			// mesh_msg.type =visualization_msgs::msg::Marker::MESH_RESOURCE;
			// mesh_msg.mesh_use_embedded_materials = true;
			// // int_marker.controls.front().markers.front().mesh_resource =  "model://turtlebot3_common/meshes/burger_base.dae";
			// mesh_msg.mesh_resource = "package://rviz_default_plugins/test_meshes/pr2-base_large.dae";
			// edges_msg_array.markers.push_back(mesh_msg);

			pub_edges_->publish(edges_msg_array);
		}

		VisualMarkerGraphNode GraphMap::getNode(NodeId id)
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