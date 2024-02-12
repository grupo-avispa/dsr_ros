/*
 * DSR BRIDGE ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#include <string>

// ROS
#include "nav2_util/node_utils.hpp"

// DSR
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/dsr_bridge.hpp"

/* Initialize the publishers and subscribers */
DSRBridge::DSRBridge(): AgentNode("dsr_bridge"){
	// Get ROS parameters
	get_params();

	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &DSRBridge::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &DSRBridge::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &DSRBridge::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &DSRBridge::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &DSRBridge::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal_by_node, this, &DSRBridge::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Publisher to the other DSR bridge
	edge_to_ros_pub_ = this->create_publisher<dsr_interfaces::msg::Edge>(
		edge_topic_, 1);
	node_to_ros_pub_ = this->create_publisher<dsr_interfaces::msg::Node>(
		node_topic_, 1);

	// Subscriber to the external DSR graph
	edge_from_ros_sub_ = this->create_subscription<dsr_interfaces::msg::Edge>(
		edge_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&DSRBridge::edge_from_ros_callback, this, std::placeholders::_1));
	node_from_ros_sub_ = this->create_subscription<dsr_interfaces::msg::Node>(
		node_topic_, 
		rclcpp::QoS(rclcpp::SystemDefaultsQoS()), 
		std::bind(&DSRBridge::node_from_ros_callback, this, std::placeholders::_1));
}

/* Initialize ROS parameters */
void DSRBridge::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "edge_topic", 
		rclcpp::ParameterValue("edges"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to publish / subscribe the edges to the DSR graph"));
	this->get_parameter("edge_topic", edge_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter edge_topic is set to: [%s]", edge_topic_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "node_topic", 
		rclcpp::ParameterValue("nodes"), 
		rcl_interfaces::msg::ParameterDescriptor() 
			.set__description("The ROS topic to publish / subscribe the nodes to the DSR graph"));
	this->get_parameter("node_topic", node_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter node_topic is set to: [%s]", node_topic_.c_str());
}

// ROS callbacks
void DSRBridge::edge_from_ros_callback(const dsr_interfaces::msg::Edge::SharedPtr msg){
	RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to edges topic");
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == this->get_name()){
		return;
	}
	// Create / Modify edge
	if (!msg->deleted){
		createEdge(msg->parent, msg->child, msg->type);
	}
	// Delete edge
	else{
		if (!G_->delete_edge(msg->parent, msg->child, msg->type)){
			RCLCPP_ERROR_STREAM(this->get_logger(), "Can't delete edge [" << msg->type << "]");
		}
	}
}

void DSRBridge::node_from_ros_callback(const dsr_interfaces::msg::Node::SharedPtr msg){
	RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to nodes topic");
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == this->get_name()){
		return;
	}

	// TODO: Actualizar a G_->get_node(msg->id) cuando se pueda modificar id del DSR
	// Check case scenario
	// Add new node
	if (!msg->deleted && !msg->updated){
		// Create node by type
		auto new_node = createNode(msg->type, msg->name);
		G_->add_or_modify_attrib_local<source_att>(new_node.value(), static_cast<std::string>(this->get_name()));
		modifyNodeAttribute(new_node.value(), msg->attributes);
		if (auto id = G_->insert_node(new_node.value()); !id.has_value()){
			RCLCPP_ERROR_STREAM(this->get_logger(), "Can't insert node");
		}
	}
	// Update current node
	else if (!msg->deleted && msg->updated){
		// Get node by name
		if (auto node = G_->get_node(msg->name); node.has_value()){
			G_->add_or_modify_attrib_local<source_att>(node.value(), static_cast<std::string>(this->get_name()));
			modifyNodeAttribute(node.value(), msg->attributes);
			G_->update_node(node.value());
		}else{
			RCLCPP_ERROR_STREAM(this->get_logger(), "The node [" << msg->id << "] doesn't exists");
		}
	}
	// Delete node
	else if (msg->deleted){
		if (auto node = G_->get_node(msg->name); node.has_value()){
			G_->delete_node(node.value().name());
		}else{
			RCLCPP_ERROR_STREAM(this->get_logger(), "The node [" << msg->id << "] doesn't exists");
		}
	}
}

std::optional<DSR::Node> DSRBridge::createNode(std::string nodeType, std::string nodeName){
	DSR::Node newNode;
	if (nodeType == "robot") {
		newNode = DSR::Node::create<robot_node_type>(nodeName);
	} else if (nodeType == "battery") {
		newNode = DSR::Node::create<battery_node_type>(nodeName);
	} else if (nodeType == "person") {
		newNode = DSR::Node::create<person_node_type>(nodeName);
	} else if (nodeType == "navigation") {
		newNode = DSR::Node::create<navigation_node_type>(nodeName);
	} else if (nodeType == "move") {
		newNode = DSR::Node::create<move_node_type>(nodeName);
	} else if (nodeType == "say") {
		newNode = DSR::Node::create<say_node_type>(nodeName);
	} else if (nodeType == "play") {
		newNode = DSR::Node::create<play_node_type>(nodeName);
	} else if (nodeType == "use_case") {
		newNode = DSR::Node::create<use_case_node_type>(nodeName);
	} else if (nodeType == "show") {
		newNode = DSR::Node::create<show_node_type>(nodeName);
	} else {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Node with type '" << nodeType << "' not valid.");
		return{};
	}
	return newNode;
}

std::optional<DSR::Edge> DSRBridge::createEdge(uint64_t from, uint64_t to, const std::string &type){
	DSR::Edge newEdge;
	if (type == "stopped") {
		newEdge = DSR::Edge::create<stopped_edge_type>(from, to);
	} else if (type == "is") {
		newEdge = DSR::Edge::create<is_edge_type>(from, to);
	} else if (type == "is_performing") {
		newEdge = DSR::Edge::create<is_performing_edge_type>(from, to);
	} else if (type == "is_with") {
		newEdge = DSR::Edge::create<is_with_edge_type>(from, to);
	} else if (type == "interacting") {
		newEdge = DSR::Edge::create<interacting_edge_type>(from, to);
	} else if (type == "wants_to") {
		newEdge = DSR::Edge::create<wants_to_edge_type>(from, to);
	} else if (type == "finished") {
		newEdge = DSR::Edge::create<finished_edge_type>(from, to);
	} else if (type == "abort") {
		newEdge = DSR::Edge::create<abort_edge_type>(from, to);
	} else if (type == "aborting") {
		newEdge = DSR::Edge::create<aborting_edge_type>(from, to);
	} else if (type == "cancel") {
		newEdge = DSR::Edge::create<cancel_edge_type>(from, to);
	} else if (type == "failed") {
		newEdge = DSR::Edge::create<failed_edge_type>(from, to);
	} else if (type == "navigating") {
		newEdge = DSR::Edge::create<navigating_edge_type>(from, to);
	} else if (type == "rt") {
		auto parent_node = G_->get_node(from);
		rt_->insert_or_assign_edge_RT(parent_node.value(), to, 
			{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
		return {};
	} else {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Edge type not valid");
		return {};
	}
	return newEdge;
}

void DSRBridge::modifyNodeAttribute(DSR::Node & node, std::vector <std::string>& attributes){
	std::string attributeName, attributeValue, attributeChoice;
	for(unsigned int i = 0; i <= attributes.size() / 2; i += 2){
		attributeName = attributes[i];
		attributeValue = attributes[i+1];
		// General
		if (attributeName == "priority") {
			G_->add_or_modify_attrib_local<priority_att>(node, std::stoi(attributeValue));
		} else if (attributeName == "result_code") {
			G_->add_or_modify_attrib_local<result_code_att>(node, attributeValue);
		} else if (attributeName == "number") {
			G_->add_or_modify_attrib_local<number_att>(node, std::stoi(attributeValue));
		// Navigation
		} else if (attributeName == "pose_x") {
			G_->add_or_modify_attrib_local<pose_x_att>(node, std::stof(attributeValue));
		} else if (attributeName == "pose_y") {
			G_->add_or_modify_attrib_local<pose_y_att>(node, std::stof(attributeValue));
		} else if (attributeName == "pose_angle") {
			G_->add_or_modify_attrib_local<pose_angle_att>(node, std::stof(attributeValue));
		} else if (attributeName == "goal_x") {
			G_->add_or_modify_attrib_local<goal_x_att>(node, std::stof(attributeValue));
		} else if (attributeName == "goal_y") {
			G_->add_or_modify_attrib_local<goal_y_att>(node, std::stof(attributeValue));
		} else if (attributeName == "goal_angle") {
			G_->add_or_modify_attrib_local<goal_angle_att>(node, std::stof(attributeValue));
		} else if (attributeName == "zone") {
			G_->add_or_modify_attrib_local<zone_att>(node, attributeValue);
		} else if (attributeName == "zones") {
			G_->add_or_modify_attrib_local<zones_att>(node, attributeValue);
		// Play / say
		} else if (attributeName == "text") {
			G_->add_or_modify_attrib_local<text_att>(node, attributeValue);
		// Show
		} else if (attributeName == "interface") {
			G_->add_or_modify_attrib_local<interface_att>(node, attributeValue);
		// Battery
		} else if (attributeName == "battery_percentage") {
			G_->add_or_modify_attrib_local<battery_percentage_att>(node, std::stof(attributeValue));
		} else if (attributeName == "battery_power_supply_status") {
			G_->add_or_modify_attrib_local<battery_power_supply_status_att>(node, attributeValue);
		// Use case
		} else if (attributeName == "use_case_id") {
			G_->add_or_modify_attrib_local<use_case_id_att>(node, attributeValue);
		// Person
		} else if (attributeName == "identifier") {
			G_->add_or_modify_attrib_local<identifier_att>(node, attributeValue);
		} else if (attributeName == "safe_distance") {
			G_->add_or_modify_attrib_local<safe_distance_att>(node, std::stof(attributeValue));
		} else if (attributeName == "menu") {
			G_->add_or_modify_attrib_local<menu_att>(node, attributeValue);
		}else if (attributeName == "timestamp") {
			G_->add_or_modify_attrib_local<timestamp_att>(node, std::stoi(attributeValue));
		}else {
			RCLCPP_ERROR_STREAM(this->get_logger(), "Attribute with type '" << attributeName 
								<< "' not valid." );
			return;
		}
	}
}

// DSR callbacks
void DSRBridge::node_updated(std::uint64_t id, const std::string &type){
	RCLCPP_INFO(this->get_logger(), "Received a new node of type %s from DSR", type.c_str());
	// Get the node from the DSR graph
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value()); 
			(source.has_value() && source != this->get_name()) || !source.has_value()){
			// Create the message
			dsr_interfaces::msg::Node msg;
			msg.header.stamp = this->now();
			msg.header.frame_id = this->get_name();
			msg.id = id;
			msg.name = dsr_node.value().name();
			msg.type = dsr_node.value().type();
			msg.updated = false;
			msg.deleted = false;
			// Get all the attributes
			for (const auto &attribute : dsr_node.value().attrs()){
				std::string att_value;
				switch (attribute.second.value().index()) {
					case 0:
						att_value = std::get<std::string>(attribute.second.value());
						break;
					case 1:
						att_value = std::to_string(std::get<int32_t>(attribute.second.value()));
						break;
					case 2:
						att_value = std::to_string(std::get<float>(attribute.second.value()));
						break;
					case 4:
						att_value = std::get<bool>(attribute.second.value()) ? "true" : "false";
						break;
					case 6:
						att_value = std::to_string(std::get<uint32_t>(attribute.second.value()));
						break;
					case 7:
						att_value = std::to_string(std::get<uint64_t>(attribute.second.value()));
						break;
					case 8:
						att_value = std::to_string(std::get<double>(attribute.second.value()));
						break;
				}
				msg.attributes.push_back(attribute.first);
				msg.attributes.push_back(att_value);
			}
			// Publish the message
			RCLCPP_INFO(this->get_logger(), 
				"Sending new node of type %s to the ROS bridge", type.c_str());
			node_to_ros_pub_->publish(msg);
		}
	}
}

void DSRBridge::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){
	// Get the node from the DSR graph
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value()); 
			(source.has_value() && source != this->get_name()) || !source.has_value()){
			// Create the message
			dsr_interfaces::msg::Node msg;
			msg.header.stamp = this->now();
			msg.header.frame_id = this->get_name();
			msg.id = id;
			msg.name = dsr_node.value().name();
			msg.type = dsr_node.value().type();
			msg.updated = true;
			msg.deleted = false;
			// Get all the attributes
			for (const auto &attribute : dsr_node.value().attrs()){
				std::string att_value;
				switch (attribute.second.value().index()) {
					case 0:
						att_value = std::get<std::string>(attribute.second.value());
						break;
					case 1:
						att_value = std::to_string(std::get<int32_t>(attribute.second.value()));
						break;
					case 2:
						att_value = std::to_string(std::get<float>(attribute.second.value()));
						break;
					case 4:
						att_value = std::get<bool>(attribute.second.value()) ? "true" : "false";
						break;
					case 6:
						att_value = std::to_string(std::get<uint32_t>(attribute.second.value()));
						break;
					case 7:
						att_value = std::to_string(std::get<uint64_t>(attribute.second.value()));
						break;
					case 8:
						att_value = std::to_string(std::get<double>(attribute.second.value()));
						break;
				}
				msg.attributes.push_back(attribute.first);
				msg.attributes.push_back(att_value);
			}
			// Publish the message
			node_to_ros_pub_->publish(msg);
		}
	}
}

void DSRBridge::edge_updated(std::uint64_t from, std::uint64_t to, const std::string &type){
	// Create the message
	dsr_interfaces::msg::Edge msg;
	msg.header.stamp = this->now();
	msg.header.frame_id = this->get_name();
	msg.parent = from;
	msg.child = to;
	msg.type = type;
	// Publish the message
	edge_to_ros_pub_->publish(msg);
}

void DSRBridge::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
}

void DSRBridge::node_deleted(const DSR::Node &node){
	// Create the message
	dsr_interfaces::msg::Node msg;
	msg.header.stamp = this->now();
	msg.header.frame_id = this->get_name();
	msg.id = node.id();
	msg.name = node.name();
	msg.deleted = true;
	// Publish the message
	node_to_ros_pub_->publish(msg);
}

void DSRBridge::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	// Create the message
	dsr_interfaces::msg::Edge msg;
	msg.header.stamp = this->now();
	msg.header.frame_id = this->get_name();
	msg.parent = from;
	msg.child = to;
	msg.type = edge_tag;
	msg.deleted = true;
	// Publish the message
	edge_to_ros_pub_->publish(msg);
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<DSRBridge>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}