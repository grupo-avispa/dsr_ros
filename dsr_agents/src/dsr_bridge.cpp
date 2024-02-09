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
		&DSR::DSRGraph::del_node_signal, this, &DSRBridge::node_deleted);

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
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == this->get_name()){
		return;
	}

	// Create the header
	std_msgs::msg::Header header;
	header.stamp = this->now();
	header.frame_id = this->get_name();

}

void DSRBridge::node_from_ros_callback(const dsr_interfaces::msg::Node::SharedPtr msg){
	// The message comes from the same name, ignore it
	if (msg->header.frame_id == this->get_name()){
		return;
	}

	// Create the header
	std_msgs::msg::Header header;
	header.stamp = this->now();
	header.frame_id = this->get_name();


}

// DSR callbacks
void DSRBridge::node_updated(std::uint64_t id, const std::string &type){
	// Get the node from the DSR graph
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		// Create the message
		dsr_interfaces::msg::Node msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = this->get_name();
		msg.id = id;
		msg.type = dsr_node.value().type();
		msg.updated = false;
		// Get all the attributes
		for (const auto &attribute : dsr_node.value().attrs()){
			std::string att;
			if (std::is_same_v<decltype(attribute.second.value()), int>){
				att = std::to_string(std::get<int>(attribute.second.value()));
			}else if (std::is_same_v<decltype(attribute.second.value()), double>){
				att = std::to_string(std::get<double>(attribute.second.value()));
			}else if (std::is_same_v<decltype(attribute.second.value()), bool>){
				att = std::get<bool>(attribute.second.value()) ? "true" : "false";
			}else if (std::is_same_v<decltype(attribute.second.value()), std::string>){
				att = std::get<std::string>(attribute.second.value());
			}
			msg.attributes.push_back(attribute.first);
			msg.attributes.push_back(att);
		}
		// Publish the message
		node_to_ros_pub_->publish(msg);
	}
}

void DSRBridge::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){
	// Get the node from the DSR graph
	if (auto dsr_node = G_->get_node(id); dsr_node.has_value()){
		// Create the message
		dsr_interfaces::msg::Node msg;
		msg.header.stamp = this->now();
		msg.header.frame_id = this->get_name();
		msg.id = id;
		msg.type = dsr_node.value().type();
		msg.updated = true;
		// Get all the attributes
		for (const auto &attribute : att_names){
			auto search = dsr_node.value().attrs().find(attribute);
			if (search != dsr_node.value().attrs().end()){
				std::string att;
				if (std::is_same_v<decltype(search->second.value()), int>){
					att = std::to_string(std::get<int>(search->second.value()));
				}else if (std::is_same_v<decltype(search->second.value()), double>){
					att = std::to_string(std::get<double>(search->second.value()));
				}else if (std::is_same_v<decltype(search->second.value()), bool>){
					att = std::get<bool>(search->second.value()) ? "true" : "false";
				}else if (std::is_same_v<decltype(search->second.value()), std::string>){
					att = std::get<std::string>(search->second.value());
				}
				msg.attributes.push_back(search->first);
				msg.attributes.push_back(att);
			}
		}
		// Publish the message
		node_to_ros_pub_->publish(msg);
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

void DSRBridge::node_deleted(std::uint64_t id){
	// Create the message
	dsr_interfaces::msg::Node msg;
	msg.header.stamp = this->now();
	msg.header.frame_id = this->get_name();
	msg.id = id;
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