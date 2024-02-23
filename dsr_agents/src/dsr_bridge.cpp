/*
 * DSR BRIDGE ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * Copyright (c) 2024 Óscar Pons Fernández <oscarpf22@gmail.com>
 * Copyright (c) 2024 José Galeas Merchan <jgaleas1999@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#include <string>

// ROS
#include "nav2_util/node_utils.hpp"
#include "nav2_util/string_utils.hpp"

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
	// Create or update the current edge
	if (!msg->deleted){
		if (auto edge = G_->get_edge(msg->parent, msg->child, msg->type); edge.has_value()){
			modify_attributes(edge.value(), msg->attributes);
			if (!G_->insert_or_assign_edge(edge.value())) {
				RCLCPP_ERROR_STREAM(this->get_logger(), "Error updating [" 
					<< msg->parent.c_str() << "->" << msg->child.c_str() << "] edge of type [" 
					<< msg->type.c_str() << "] in the DSR");
			}
		}else{
			auto new_edge = create_dsr_edge(msg->parent, msg->child, msg->type);
			modify_attributes(new_edge.value(), msg->attributes);
			if (!G_->insert_or_assign_edge(new_edge.value())){
				RCLCPP_ERROR_STREAM(this->get_logger(), "Error inserting [" 
					<< msg->parent.c_str() << "->" << msg->child.c_str() << "] edge of type [" 
					<< msg->type.c_str() << "] in the DSR");
			}
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

	// Create or update the current node
	if (!msg->deleted){
		if (auto node = G_->get_node(msg->name); node.has_value()){
			modify_attributes(node.value(), msg->attributes);
			if (!G_->update_node(node.value())){
				RCLCPP_ERROR(this->get_logger(), "Error updating [%s] node", msg->name.c_str());
			}
		}else{
			auto new_node = create_dsr_node(msg->name, msg->type);
			modify_attributes(new_node.value(), msg->attributes);
			if (auto id = G_->insert_node(new_node.value()); !id.has_value()){
				RCLCPP_ERROR(this->get_logger(), "Error inserting [%s] node", msg->name.c_str());
			}
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
			(source.has_value() && source == source_)){
			// Create the message
			auto node_msg = create_msg_node(dsr_node.value().name(), dsr_node.value().type());
			// Get all the attributes
			for (const auto& [att_name, att_value] : dsr_node.value().attrs()){
				std::string att_str = attribute_to_string(att_value);
				node_msg.attributes.push_back(att_name);
				node_msg.attributes.push_back(att_str);
				RCLCPP_DEBUG(this->get_logger(), 
					"Attribute node [%s] = [%s]", att_name.c_str(), att_str.c_str());
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
			(source.has_value() && source.value() == source_)){
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
						"Attribute edge [%s] = [%s]", att_name.c_str(), att_value.c_str());
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
	if (auto dsr_edge = G_->get_edge(from, to, type); dsr_edge.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value()); 
			(source.has_value() && source.value() == source_)){
			// Create the message
			auto edge_msg = create_msg_edge(from, to, type);
			// Get all the attributes
			for (const auto& [att_name, att_value] : dsr_edge.value().attrs()){
				std::string att_str = attribute_to_string(att_value);
				edge_msg.attributes.push_back(att_name);
				edge_msg.attributes.push_back(att_str);
				RCLCPP_DEBUG(this->get_logger(), 
					"Attribute edge [%s] = [%s]", att_name.c_str(), att_str.c_str());
			}
			// Publish the message
			edge_to_ros_pub_->publish(edge_msg);
			RCLCPP_DEBUG_STREAM(this->get_logger(), "The edge [" 
				<< edge_msg.parent.c_str() << "->" 
				<< edge_msg.child.c_str() << "] of type ["
				<< edge_msg.type.c_str() << "] has been created in the DSR");
		}
	}
}

void DSRBridge::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
	// Filter the edges that comes from the same source
	if (auto dsr_edge = G_->get_edge(from, to, type); dsr_edge.has_value()){
		if (auto source = G_->get_attrib_by_name<source_att>(dsr_edge.value()); 
			(source.has_value() && source.value() == source_)){
			// Create the message
			auto edge_msg = create_msg_edge(from, to, type);
			// Mark the node as updated
			edge_msg.updated = true;
			// Get all the updated attributes
			for (const auto &att_name : att_names){
				auto search = dsr_edge.value().attrs().find(att_name);
				if (search != dsr_edge.value().attrs().end()){
					std::string att_value = attribute_to_string(search->second);
					edge_msg.attributes.push_back(att_name);
					edge_msg.attributes.push_back(att_value);
					RCLCPP_DEBUG(this->get_logger(), 
						"Attribute edge [%s] = [%s]", att_name.c_str(), att_value.c_str());
				}
			}
			// Publish the message
			edge_to_ros_pub_->publish(edge_msg);
			RCLCPP_DEBUG_STREAM(this->get_logger(), "Updated edge [" 
				<< edge_msg.parent.c_str() << "->" 
				<< edge_msg.child.c_str() << "] of type ["
				<< edge_msg.type.c_str() << "] successfully in the DSR");
		}
	}
}

void DSRBridge::node_deleted(const DSR::Node &node){
	// Create the message
	auto node_msg = create_msg_node(node.name(), node.type());
	// Mark the node as deleted
	node_msg.deleted = true;
	// Publish the message
	node_to_ros_pub_->publish(node_msg);
	RCLCPP_DEBUG_STREAM(this->get_logger(), 
		"The node [" << node.name() << "] of type [" 
		<< node.type() << "] has been deleted in the DSR");
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

// Converter functions
std::optional<DSR::Node> DSRBridge::create_dsr_node(std::string name, std::string type){
	if (!node_types::check_type(type)) {
		throw std::runtime_error("Error, [" + type + "] is not a valid node type");
	}
	DSR::Node new_node;
	new_node.name(name);
	new_node.type(type);
	return new_node;
}

std::optional<DSR::Edge> DSRBridge::create_dsr_edge(
	std::string from, std::string to, const std::string &type){
	if (!edge_types::check_type(type)) {
		throw std::runtime_error("Error, [" + type + "] is not a valid edge type");
	}
	DSR::Edge new_edge;
	auto parent_node = G_->get_node(from);
	auto child_node = G_->get_node(to);
	if (parent_node.has_value() && child_node.has_value()){
		new_edge.from(parent_node.value().id());
		new_edge.to(child_node.value().id());
		new_edge.type(type);
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
		edge_msg.updated = false;
		edge_msg.deleted = false;
	}
	return edge_msg;
}

// Helper functions
template <typename TYPE>
void DSRBridge::modify_attributes(TYPE & elem, std::vector <std::string>& att_str){
	for (unsigned int i = 0; i < att_str.size(); i+=2){
		std::string att_name = att_str[i];
		std::string att_value = att_str[i+1];
		DSR::Types att_type = parse_type(att_value);

		DSR::Attribute new_att;
		switch (att_type) {
			case DSR::Types::STRING:{
				new_att.value(std::string(att_value));
				break;
			}
			case DSR::Types::INT:{
				new_att.value(std::stoi(att_value));
				break;
			}
			case DSR::Types::FLOAT:{
				new_att.value(std::stof(att_value));
				break;
			}
			case DSR::Types::FLOAT_VEC:{
				std::vector<std::string> values = nav2_util::split(att_value, ',');
				std::vector<float> float_values;
				for (const auto &value: values){
					float_values.push_back(std::stof(value));
				}
				new_att.value(float_values);
				break;
			}
			case DSR::Types::BOOL:{
				new_att.value(att_value == "true");
				break;
			}
			default:
				break;
		}
		// Add the attribute to the element
		G_->runtime_checked_add_attrib_local(elem, att_name, new_att);
		RCLCPP_DEBUG(this->get_logger(), 
			"Updating attribute [%s] = [%s]", att_name.c_str(), att_value.c_str());
	}
}

std::string DSRBridge::attribute_to_string(const DSR::Attribute &att){
	std::locale::global(std::locale("C"));
	switch (att.value().index()) {
		case 0:{
			return std::get<std::string>(att.value());
		}
		case 1:{
			return std::to_string(std::get<int32_t>(att.value()));
		}
		case 2:{
			return std::to_string(std::get<float>(att.value()));
		}
		case 3:{
			std::string att_str;
			for (const auto &value: std::get<std::vector<float>>(att.value())){
				att_str = std::to_string(value) + std::string(";");
			}
			att_str.pop_back();
			return att_str;
		}
		case 4:{
			return std::get<bool>(att.value()) ? "true" : "false";
		}
		case 5:{
			std::string att_str;
			for (const auto &value: std::get<std::vector<uint8_t>>(att.value())){
				att_str = std::to_string(value) + std::string(";");
			}
			att_str.pop_back();
			return att_str;
		}
		case 6:{
			return std::to_string(std::get<uint32_t>(att.value()));
		}
		case 7:{
			return std::to_string(std::get<uint64_t>(att.value()));
		}
		case 8:{
			return std::to_string(std::get<double>(att.value()));
		}
	}
}

DSR::Types DSRBridge::parse_type(const std::string &type){
	std::istringstream iss(type);

	// Try to convert to int
	int int_result;
	iss >> int_result;
	if (!iss.fail() && iss.eof()) {
		return DSR::Types::INT;
	}
	// Clear the state of iss and reset the read pointer
	iss.clear();
	iss.seekg(0);

	// Try to convert to float
	float float_result;
	iss >> float_result;
	if (!iss.fail() && iss.eof()) {
		if (float_result == static_cast<int>(float_result)) {
			return DSR::Types::INT;
		} else {
			return DSR::Types::FLOAT;
		}
	}
	iss.clear();
	iss.seekg(0);

	// Try to convert to vector of float (separated by comma)
	char comma;
	std::vector<float> vector_result;
	while (iss >> float_result >> comma) {
		// Check if the delimitator is a comma except at the end
		if (comma != ',' && !iss.eof()) {
			return DSR::Types::STRING;
		}
		vector_result.push_back(float_result);
	}
	if (iss.eof()) {
		return DSR::Types::FLOAT_VEC;
	}
	iss.clear();
	iss.seekg(0);

	// Try to convert to bool ("true" or "false")
	bool bool_result;
	iss >> std::boolalpha >> bool_result;
	if (!iss.fail() && iss.eof()) {
		return DSR::Types::BOOL;
	}

	// In other case, return a string
	return DSR::Types::STRING;
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