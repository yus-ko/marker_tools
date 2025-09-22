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
			auto graph_handle = menu_handler_->insert("graph");

			menu_handler_->insert(graph_handle, "add edge",
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

		void GraphMap::initializeMarkerServer(const std::map<std::string, VisualMarker> &markers)
		{
			InteractiveMarkerManager::initializeMarkerServer(markers);
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
					publishGraphMap(graph_);
				}
				else
				{
					publishGraphMap(graph_);
				}
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