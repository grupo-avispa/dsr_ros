/*
 * DSR BRIDGE ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * Copyright (c) 2024 Óscar Pons Fernández <oscarpf22@gmail.com>
 * Copyright (c) 2024 José Galeas Merchan <>
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
		&DSR::DSRGraph::create_node_signal, this, &DSRBridge::node_created);
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
	RCLCPP_INFO_ONCE(this->get_logger(), 
		"Subscribed to edges topic from [%s]", msg->header.frame_id.c_str());
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == source_){
		return;
	}
	// Create or update and edge
	if (!msg->deleted){
		auto new_edge = create_dsr_edge(msg->parent, msg->child, msg->type);
		// Add the source attribute with the physical machine name
		G_->add_or_modify_attrib_local<source_att>(new_edge.value(), msg->header.frame_id);
		if (!G_->insert_or_assign_edge(new_edge.value())){
			RCLCPP_ERROR_STREAM(this->get_logger(), "Can't insert edge [" << msg->type << "]");
		}
	}
	// Delete an edge
	else{
		delete_edge(msg->parent, msg->child, msg->type);
	}
}

void DSRBridge::node_from_ros_callback(const dsr_interfaces::msg::Node::SharedPtr msg){
	RCLCPP_INFO_ONCE(this->get_logger(), 
		"Subscribed to nodes topic from [%s]", msg->header.frame_id.c_str());
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == source_){
		return;
	}

	// Create a new node
	if (!msg->deleted && !msg->updated){
		auto new_node = create_dsr_node(msg->name, msg->type);
		modify_node_attributes(new_node.value(), msg->attributes);
		if (auto id = G_->insert_node(new_node.value()); !id.has_value()){
			RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", msg->name.c_str());
		}
	}
	// Update the current node
	else if (!msg->deleted && msg->updated){
		// Get node by name
		if (auto node = G_->get_node(msg->name); node.has_value()){
			modify_node_attributes(node.value(), msg->attributes);
			G_->update_node(node.value());
		}else{
			RCLCPP_ERROR(this->get_logger(), "Error updating [%s] node", msg->name.c_str());
		}
	}
	// Delete the node
	else if (msg->deleted){
		if (auto node = G_->get_node(msg->name); node.has_value()){
			G_->delete_node(node.value().name());
		}else{
			RCLCPP_ERROR(this->get_logger(), "Error deleting [%s] node", msg->name.c_str());
		}
	}
}

// DSR callbacks
void DSRBridge::node_created(std::uint64_t id, const std::string &type){
	// Filter the edges that comes from the same source
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value()); 
			(source.has_value() && source == source_) || !source.has_value()){
			// Create the message
			auto node_msg = create_msg_node(dsr_node.value().name(), dsr_node.value().type());
			// Get all the attributes
			for (const auto& [att_name, att_value] : dsr_node.value().attrs()){
				std::string att_str = attribute_to_string(att_value);
				node_msg.attributes.push_back(att_name);
				node_msg.attributes.push_back(att_str);
				RCLCPP_DEBUG(this->get_logger(), 
					"Attribute [%s] = [%s]", att_name.c_str(), att_str.c_str());
			}
			// Publish the message
			node_to_ros_pub_->publish(node_msg);
			RCLCPP_DEBUG(this->get_logger(), 
				"Inserted [%s] node successfully of type [%s] in the DSR", 
				node_msg.name.c_str(), node_msg.type.c_str());
		}
	}
}

void DSRBridge::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){
	// Filter the edges that comes from the same source
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_node.value()); 
			(source.has_value() && source.value() == source_) || !source.has_value()){
			// Create the message
			auto node_msg = create_msg_node(dsr_node.value().name(), dsr_node.value().type());
			// Mark the node as updated
			node_msg.updated = true;
			// Get all the updated attributes
			for (const auto &att_name : att_names){
				auto search = dsr_node.value().attrs().find(att_name);
				if (search != dsr_node.value().attrs().end()){
					std::string att_value = attribute_to_string(search->second);
					node_msg.attributes.push_back(att_name);
					node_msg.attributes.push_back(att_value);
					RCLCPP_DEBUG(this->get_logger(), 
						"Attribute [%s] = [%s]", att_name.c_str(), att_value.c_str());
				}
			}
			// Publish the message
			node_to_ros_pub_->publish(node_msg);
			RCLCPP_DEBUG(this->get_logger(), 
				"Updated [%s] node successfully of type [%s] in the DSR", 
				node_msg.name.c_str(), node_msg.type.c_str());
		}
	}
}

void DSRBridge::edge_updated(std::uint64_t from, std::uint64_t to, const std::string &type){
	// Filter the edges that comes from the same source
	auto edge = G_->get_edge(from, to, type);
	auto source = G_->get_attrib_by_name<source_att>(edge.value()); 
	if ((source.has_value() && source.value() == source_) || !source.has_value()){
		// Create the message
		auto edge_msg = create_msg_edge(from, to, type);
		// Publish the message
		edge_to_ros_pub_->publish(edge_msg);
		RCLCPP_DEBUG_STREAM(this->get_logger(), "The edge [" 
			<< edge_msg.parent.c_str() << "->" 
			<< edge_msg.child.c_str() << "] of type ["
			<< edge_msg.type.c_str() << "] has been created in the DSR");
	}
}

void DSRBridge::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
}

void DSRBridge::node_deleted(const DSR::Node &node){
	// Create the message
	auto node_msg = create_msg_node(node.name(), node.type());
	// Mark the node as deleted
	node_msg.deleted = true;
	// Publish the message
	node_to_ros_pub_->publish(node_msg);
	RCLCPP_DEBUG_STREAM(this->get_logger(), 
		"The node [" << node.name() << "] of type [" << node.type() << "] has been deleted in the DSR");
}

void DSRBridge::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	// Create the message
	auto edge_msg = create_msg_edge(from, to, edge_tag);
	// Mark the edge as deleted
	edge_msg.deleted = true;
	// Publish the message
	edge_to_ros_pub_->publish(edge_msg);
	RCLCPP_DEBUG_STREAM(this->get_logger(), "The edge [" 
		<< edge_msg.parent.c_str() << "->" 
		<< edge_msg.child.c_str() << "] of type ["
		<< edge_msg.type.c_str() << "] has been deleted in the DSR");
}

// Helper functions
std::optional<DSR::Node> DSRBridge::create_dsr_node(std::string name, std::string type){
	DSR::Node new_node;
	if (type == "robot") {
		new_node = DSR::Node::create<robot_node_type>(name);
	} else if (type == "battery") {
		new_node = DSR::Node::create<battery_node_type>(name);
	} else if (type == "person") {
		new_node = DSR::Node::create<person_node_type>(name);
	} else if (type == "navigation") {
		new_node = DSR::Node::create<navigation_node_type>(name);
	} else if (type == "move") {
		new_node = DSR::Node::create<move_node_type>(name);
	} else if (type == "say") {
		new_node = DSR::Node::create<say_node_type>(name);
	} else if (type == "play") {
		new_node = DSR::Node::create<play_node_type>(name);
	} else if (type == "use_case") {
		new_node = DSR::Node::create<use_case_node_type>(name);
	} else if (type == "show") {
		new_node = DSR::Node::create<show_node_type>(name);
	} else if (type == "update_bbdd") {
		new_node = DSR::Node::create<update_bbdd_node_type>(name);
	} else {
		RCLCPP_ERROR_STREAM(this->get_logger(), "Node with type '" << name << "' not valid.");
		return{};
	}
	return new_node;
}

std::optional<DSR::Edge> DSRBridge::create_dsr_edge(
	std::string from, std::string to, const std::string &type){
	DSR::Edge new_edge;
	auto parent_node = G_->get_node(from);
	auto child_node = G_->get_node(to);
	if (parent_node.has_value() && child_node.has_value()){
		if (type == "stopped") {
			new_edge = DSR::Edge::create<stopped_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "is") {
			new_edge = DSR::Edge::create<is_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "is_performing") {
			new_edge = DSR::Edge::create<is_performing_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "is_with") {
			new_edge = DSR::Edge::create<is_with_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "interacting") {
			new_edge = DSR::Edge::create<interacting_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "wants_to") {
			new_edge = DSR::Edge::create<wants_to_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "finished") {
			new_edge = DSR::Edge::create<finished_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "abort") {
			new_edge = DSR::Edge::create<abort_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "aborting") {
			new_edge = DSR::Edge::create<aborting_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "cancel") {
			new_edge = DSR::Edge::create<cancel_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "failed") {
			new_edge = DSR::Edge::create<failed_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "navigating") {
			new_edge = DSR::Edge::create<navigating_edge_type>(
				parent_node.value().id(), child_node.value().id());
		} else if (type == "rt") {
			rt_->insert_or_assign_edge_RT(parent_node.value(), child_node.value().id(), 
				{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
			return {};
		} else {
			RCLCPP_ERROR_STREAM(this->get_logger(), "Edge type not valid");
			return {};
		}
	}
	return new_edge;
}

dsr_interfaces::msg::Node DSRBridge::create_msg_node(std::string name, std::string type){
	dsr_interfaces::msg::Node node_msg;
	node_msg.header.stamp = this->now();
	node_msg.header.frame_id = source_;
	node_msg.name = name;
	node_msg.type = type;
	node_msg.updated = false;
	node_msg.deleted = false;
	return node_msg;
}

dsr_interfaces::msg::Edge DSRBridge::create_msg_edge(
	std::uint64_t from, std::uint64_t to, const std::string &type){
	dsr_interfaces::msg::Edge edge_msg;
	auto parent_node = G_->get_node(from);
	auto child_node = G_->get_node(to);
	if (parent_node.has_value() && child_node.has_value()){
		edge_msg.header.stamp = this->now();
		edge_msg.header.frame_id = source_;
		edge_msg.parent = parent_node.value().name();
		edge_msg.child = child_node.value().name();
		edge_msg.type = type;
		edge_msg.deleted = false;
	}
	return edge_msg;
}

void DSRBridge::modify_node_attributes(DSR::Node & node, std::vector<std::string>& att_str){
	/*std::map<std::string, DSR::Attribute> attributes;
	for (unsigned int i = 0; i <= att_str.size() / 2; i += 2){
		std::string att_name = att_str[i];
		std::string att_value = att_str[i+1];
		//attributes[att_name] = att_value;
		DSR::Attribute::get_valtype(att_value);
	}*/

	for (unsigned int i = 0; i <= att_str.size() / 2; i += 2){
		std::string att_name = att_str[i];
		std::string att_value = att_str[i+1];
		// General
		if (att_name == "priority") {
			G_->add_or_modify_attrib_local<priority_att>(node, std::stoi(att_value));
		} else if (att_name == "result_code") {
			G_->add_or_modify_attrib_local<result_code_att>(node, att_value);
		} else if (att_name == "number") {
			G_->add_or_modify_attrib_local<number_att>(node, std::stoi(att_value));
		} else if (att_name == "source") {
			G_->add_or_modify_attrib_local<source_att>(node, att_value);
		} else if (att_name == "pos_x") {
			G_->add_or_modify_attrib_local<pos_x_att>(node, std::stof(att_value));
		} else if (att_name == "pos_y") {
			G_->add_or_modify_attrib_local<pos_y_att>(node, std::stof(att_value));
		// Navigation
		} else if (att_name == "pose_x") {
			G_->add_or_modify_attrib_local<pose_x_att>(node, std::stof(att_value));
		} else if (att_name == "pose_y") {
			G_->add_or_modify_attrib_local<pose_y_att>(node, std::stof(att_value));
		} else if (att_name == "pose_angle") {
			G_->add_or_modify_attrib_local<pose_angle_att>(node, std::stof(att_value));
		} else if (att_name == "goal_x") {
			G_->add_or_modify_attrib_local<goal_x_att>(node, std::stof(att_value));
		} else if (att_name == "goal_y") {
			G_->add_or_modify_attrib_local<goal_y_att>(node, std::stof(att_value));
		} else if (att_name == "goal_angle") {
			G_->add_or_modify_attrib_local<goal_angle_att>(node, std::stof(att_value));
		} else if (att_name == "zone") {
			G_->add_or_modify_attrib_local<zone_att>(node, att_value);
		} else if (att_name == "zones") {
			G_->add_or_modify_attrib_local<zones_att>(node, att_value);
		// Play / say
		} else if (att_name == "text") {
			G_->add_or_modify_attrib_local<text_att>(node, att_value);
		// Show
		} else if (att_name == "interface") {
			G_->add_or_modify_attrib_local<interface_att>(node, att_value);
		// Battery
		} else if (att_name == "battery_percentage") {
			G_->add_or_modify_attrib_local<battery_percentage_att>(node, std::stof(att_value));
		} else if (att_name == "battery_power_supply_status") {
			G_->add_or_modify_attrib_local<battery_power_supply_status_att>(node, att_value);
		// Use case
		} else if (att_name == "use_case_id") {
			G_->add_or_modify_attrib_local<use_case_id_att>(node, att_value);
		// Person
		} else if (att_name == "identifier") {
			G_->add_or_modify_attrib_local<identifier_att>(node, att_value);
		} else if (att_name == "safe_distance") {
			G_->add_or_modify_attrib_local<safe_distance_att>(node, std::stof(att_value));
		} else if (att_name == "menu") {
			G_->add_or_modify_attrib_local<menu_att>(node, att_value);
		}else if (att_name == "timestamp") {
			G_->add_or_modify_attrib_local<timestamp_att>(node, std::stoi(att_value));
		}else {
			RCLCPP_ERROR_STREAM(this->get_logger(), 
				"Attribute with type '" << att_name << "' not valid." );
			return;
		}
	}
}

std::string DSRBridge::attribute_to_string(const DSR::Attribute &att){
	switch (att.value().index()) {
		case 0:
			return std::get<std::string>(att.value());
		case 1:
			return std::to_string(std::get<int32_t>(att.value()));
		case 2:
			return std::to_string(std::get<float>(att.value()));
		case 4:
			return std::get<bool>(att.value()) ? "true" : "false";
		case 6:
			return std::to_string(std::get<uint32_t>(att.value()));
		case 7:
			return std::to_string(std::get<uint64_t>(att.value()));
		case 8:
			return std::to_string(std::get<double>(att.value()));
	}
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