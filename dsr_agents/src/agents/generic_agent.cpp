/*
 * GENERIC AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

// C++
#include <chrono>
#include <thread>

// ROS
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// DSR
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/agents/generic_agent.hpp"

/* Initialize the publishers and subscribers */
genericAgent::genericAgent(): AgentNode("generic_agent"){
	// Get ROS parameters
	get_params();

	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &genericAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &genericAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &genericAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &genericAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &genericAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &genericAgent::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Subscriber to the topic with a generic subscription
	auto data = rclcpp::Node::get_topic_names_and_types();
	for (auto type : data[ros_topic_]){
		generic_sub_ = create_generic_subscription(
			ros_topic_, 
			type, 
			rclcpp::QoS(rclcpp::SensorDataQoS()),
			std::bind(&genericAgent::serial_callback, this, std::placeholders::_1));
	}
}

/* Initialize ROS parameters */
void genericAgent::get_params(){
	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "ros_topic", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter ros_topic is set to: [%s]", ros_topic_.c_str());

	// DSR parameters
	nav2_util::declare_parameter_if_not_declared(this, "dsr_node_name", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the node in the DSR graph"));
	this->get_parameter("dsr_node_name", dsr_node_name_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter dsr_node is set to: [%s]", dsr_node_name_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "dsr_parent_node_name", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the parent node in the DSR graph"));
	this->get_parameter("dsr_parent_node_name", dsr_parent_node_name_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter dsr_parent_node_name is set to: [%s]", dsr_parent_node_name_.c_str());

	// Default DSR node name to ROS topic
	dsr_node_name_ = dsr_node_name_.empty() ? ros_topic_ : dsr_node_name_;
}

template <> 
void genericAgent::modify_attributes<sensor_msgs::msg::BatteryState>(
	std::optional<DSR::Node> &node, const sensor_msgs::msg::BatteryState &msg){
	// Modify the attributes of the node
	G_->add_or_modify_attrib_local<battery_voltage_att>(node.value(), msg.voltage);
	G_->add_or_modify_attrib_local<battery_temperature_att>(node.value(), msg.temperature);
	G_->add_or_modify_attrib_local<battery_current_att>(node.value(), msg.current);
	G_->add_or_modify_attrib_local<battery_charge_att>(node.value(), msg.charge);
	G_->add_or_modify_attrib_local<battery_capacity_att>(node.value(), msg.capacity);
	G_->add_or_modify_attrib_local<battery_design_capacity_att>(node.value(), msg.design_capacity);
	G_->add_or_modify_attrib_local<battery_percentage_att>(node.value(), msg.percentage);
	G_->add_or_modify_attrib_local<battery_power_supply_status_att>(node.value(), 
		static_cast<int>(msg.power_supply_status));
	G_->add_or_modify_attrib_local<battery_power_supply_health_att>(node.value(), 
		static_cast<int>(msg.power_supply_health));
	G_->add_or_modify_attrib_local<battery_power_supply_technology_att>(node.value(), 
		static_cast<int>(msg.power_supply_technology));
	G_->add_or_modify_attrib_local<battery_present_att>(node.value(), msg.present);
	G_->add_or_modify_attrib_local<battery_cell_voltage_att>(node.value(), msg.cell_voltage);
	G_->add_or_modify_attrib_local<battery_cell_temperature_att>(node.value(), msg.cell_temperature);
	G_->add_or_modify_attrib_local<battery_location_att>(node.value(), msg.location);
	G_->add_or_modify_attrib_local<battery_serial_number_att>(node.value(), msg.serial_number);
	// Print the attributes of the node
	RCLCPP_DEBUG(this->get_logger(), "%s node updated with attributes:", node.value().name().c_str());
	for (auto &[key, value] : node.value().attrs()){
		RCLCPP_DEBUG(this->get_logger(), "Attribute [%s] = [%s]", key.c_str(), value.value());
	}
}

template <> 
void genericAgent::modify_attributes<sensor_msgs::msg::Image>(
	std::optional<DSR::Node> &node, const sensor_msgs::msg::Image &msg){
	// Modify the attributes of the node depending the type of the image
	if (msg.encoding == sensor_msgs::image_encodings::RGB8){
		G_->add_or_modify_attrib_local<cam_rgb_att>(node.value(), msg.data);
		G_->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), static_cast<int>(msg.height));
		G_->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), static_cast<int>(msg.width));
	}else if (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1){
		G_->add_or_modify_attrib_local<cam_depth_att>(node.value(), msg.data);
		G_->add_or_modify_attrib_local<cam_depth_height_att>(node.value(), static_cast<int>(msg.height));
		G_->add_or_modify_attrib_local<cam_depth_width_att>(node.value(), static_cast<int>(msg.width));
	}
	// Print the attributes of the node
	RCLCPP_DEBUG(this->get_logger(), 
		"Update [%s] node with attributes: ", node.value().name().c_str());
	for (auto &[key, value] : node.value().attrs()){
		RCLCPP_DEBUG(this->get_logger(), "Attribute [%s] = [%s]", key.c_str(), value.value());
	}
}

template <> 
void genericAgent::modify_attributes<sensor_msgs::msg::LaserScan>(
	std::optional<DSR::Node> &node, const sensor_msgs::msg::LaserScan &msg){
	
	// Convert from ROS to DSR
	std::vector<float> angles(msg.ranges.size());
	angles[0] = msg.angle_min;
	for (uint i = 1; i < msg.ranges.size()-1; i++){
		angles[i] = angles[i-1] + msg.angle_increment;
	}
	angles[msg.ranges.size()-1] = msg.angle_max;
	
	// Modify the attributes of the node
	G_->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
	G_->add_or_modify_attrib_local<laser_dists_att>(node.value(), msg.ranges);
	// Print the attributes of the node
	RCLCPP_DEBUG(this->get_logger(), 
		"Update [%s] node with attributes: ", node.value().name().c_str());
	for (auto &[key, value] : node.value().attrs()){
		RCLCPP_DEBUG(this->get_logger(), "Attribute [%s] = [%s]", key.c_str(), value.value());
	}
}

template <typename ROS_TYPE, typename NODE_TYPE, typename EDGE_TYPE> 
void genericAgent::deserialize_and_update_attributes(
	const std::shared_ptr<rclcpp::SerializedMessage> msg, 
	const std::string &node_name, const std::string &parent_name){
	// Deserialize a message to ROS_TYPE
	ROS_TYPE ros_msg;
	auto serializer = rclcpp::Serialization<ROS_TYPE>();
	serializer.deserialize_message(msg.get(), &ros_msg);

	// Check if the node exists in the graph
	auto dsr_node = G_->get_node(node_name);
	if (!dsr_node.has_value()){
		// If the parent name is empty, the parent is the frame_id of the ROS message
		auto new_parent_name = parent_name.empty() ? get_frame_id<ROS_TYPE>(ros_msg) : parent_name;
		add_node<NODE_TYPE, EDGE_TYPE>(node_name, new_parent_name);
	}
	// Update the attributes of the node
	dsr_node = G_->get_node(node_name);
	modify_attributes<ROS_TYPE>(dsr_node, ros_msg);
	G_->update_node(dsr_node.value());
}

void genericAgent::serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg){
	// In order to deserialize the message we have to manually create a ROS2
	// message in which we want to convert the serialized data.
	auto data = rclcpp::Node::get_topic_names_and_types();
	const std::string topic_type = data[ros_topic_][0];
	RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to topic [%s] of type [%s]", 
						ros_topic_.c_str(), topic_type.c_str());

	// TODO: Replace 'has_edge_type' to a type according to (sensor, actuator, navigation, etc)
	if (topic_type == "sensor_msgs/msg/BatteryState"){
		deserialize_and_update_attributes<sensor_msgs::msg::BatteryState, 
											battery_node_type,
											has_edge_type>(msg, dsr_node_name_, dsr_parent_node_name_);
	}else if (topic_type == "sensor_msgs/msg/Image"){
		deserialize_and_update_attributes<sensor_msgs::msg::Image, 
											rgbd_node_type,
											has_edge_type>(msg, dsr_node_name_, dsr_parent_node_name_);
	}else if (topic_type == "sensor_msgs/msg/LaserScan"){
		deserialize_and_update_attributes<sensor_msgs::msg::LaserScan, 
											laser_node_type,
											has_edge_type>(msg, dsr_node_name_, dsr_parent_node_name_);
	}else{
		RCLCPP_WARN_ONCE(this->get_logger(), 
			"Received message of type [%s]. Unknown for the DSR.", topic_type.c_str());
	}
}

void genericAgent::node_updated(std::uint64_t id, const std::string &type){
	// Only for debugging
	/*if (type == "battery"){
		if (auto node = G_->get_node(id); node.has_value()){
			auto voltage = G_->get_attrib_by_name<battery_voltage_att>(node.value());
			if (voltage.has_value()){
				RCLCPP_DEBUG(this->get_logger(), "Battery voltage is [%f]", voltage.value());
			}
		}
	}*/
}

void genericAgent::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){

}

void genericAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){

}

void genericAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){

}

void genericAgent::node_deleted(std::uint64_t id){

}

void genericAgent::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){

}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<genericAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}