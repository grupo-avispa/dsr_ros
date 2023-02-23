/*
 * DSR AGENT ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

// ROS
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "dsr_agent/dsr_agent.hpp"
#include "dsr_agent/ros_attr_name.hpp"

/* Initialize the publishers and subscribers */
dsrAgent::dsrAgent(): Node("dsr_agent"){
	// Get ROS parameters
	get_params();

	// Create graph
	G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "");

	// Add connection signals
	QObject::connect(G_.get(), &DSR::DSRGraph::update_node_signal, this, &dsrAgent::node_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_node_attr_signal, this, &dsrAgent::node_attributes_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_edge_signal, this, &dsrAgent::edge_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &dsrAgent::edge_attributes_updated);
	QObject::connect(G_.get(), &DSR::DSRGraph::del_edge_signal, this, &dsrAgent::edge_deleted);
	QObject::connect(G_.get(), &DSR::DSRGraph::del_node_signal, this, &dsrAgent::node_deleted);

	// Subscriber to the topic with a generic subscription
	auto data = rclcpp::Node::get_topic_names_and_types();
	for (auto type : data[ros_topic_]){
		generic_sub_ = create_generic_subscription(
						ros_topic_, 
						type, 
						rclcpp::QoS(rclcpp::SensorDataQoS()),
						std::bind(&dsrAgent::serial_callback, this, std::placeholders::_1)
						);
	}
}

dsrAgent::~dsrAgent() {
	// TODO: Save the log into a file
	//G_->write_to_json_file("./"+agent_name+".json");
	G_.reset();
}

/* Initialize ROS parameters */
void dsrAgent::get_params(){
	// Agent parameters
	nav2_util::declare_parameter_if_not_declared(this, "agent_name", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The agent name to publish to"));
	this->get_parameter("agent_name", agent_name_);
	RCLCPP_INFO(this->get_logger(), "The parameter agent_name is set to: [%s]", agent_name_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "agent_id", rclcpp::ParameterValue(0), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The id of the agent")
							.set__integer_range({rcl_interfaces::msg::IntegerRange()
								.set__from_value(0)
								.set__to_value(1000)
								.set__step(1)}
								));
	this->get_parameter("agent_id", agent_id_);
	RCLCPP_INFO(this->get_logger(), "The parameter agent_id is set to: [%d]", agent_id_);

	// ROS parameters
	nav2_util::declare_parameter_if_not_declared(this, "ros_topic", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The ROS topic to subscribe to"));
	this->get_parameter("ros_topic", ros_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter ros_topic is set to: [%s]", ros_topic_.c_str());

	// DSR parameters
	nav2_util::declare_parameter_if_not_declared(this, "dsr_node_name", rclcpp::ParameterValue(""), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The name of the node in the DSR graph"));
	this->get_parameter("dsr_node_name", dsr_node_name_);
	RCLCPP_INFO(this->get_logger(), "The parameter dsr_node is set to: [%s]", dsr_node_name_.c_str());

	nav2_util::declare_parameter_if_not_declared(this, "dsr_parent_node_name", rclcpp::ParameterValue("robot"), 
							rcl_interfaces::msg::ParameterDescriptor()
							.set__description("The name of the parent node in the DSR graph"));
	this->get_parameter("dsr_parent_node_name", dsr_parent_node_name_);
	RCLCPP_INFO(this->get_logger(), "The parameter dsr_parent_node_name is set to: [%s]", dsr_parent_node_name_.c_str());
}

template <typename NODE_TYPE> 
std::optional<uint64_t> dsrAgent::create_and_insert_node(const std::string &name){
	RCLCPP_ERROR(this->get_logger(), "Node [%s] not found", name.c_str());
	auto new_dsr_node = DSR::Node::create<NODE_TYPE>(name);
	auto id = G_->insert_node(new_dsr_node);
	if (id.has_value()){
		RCLCPP_INFO(this->get_logger(), "Inserted [%s] node successfully with id [%u]", name.c_str(), id.value());
	}
	return id;
}

template <typename EDGE_TYPE> 
void dsrAgent::create_and_insert_edge(uint64_t from, uint64_t to){
	RCLCPP_ERROR_STREAM(this->get_logger(), "Edge [" << from << "->" << to << "] not found");
	auto new_edge = DSR::Edge::create<EDGE_TYPE>(from, to);
	if (G_->insert_or_assign_edge(new_edge)){
		RCLCPP_INFO_STREAM(this->get_logger(), "Inserted new edge [" << from << "->" << to <<
			"] of type [" << new_edge.type().c_str() << "]");
	}else{
		RCLCPP_ERROR_STREAM(this->get_logger(), "The edge [" << from << "->" << to <<
			"] of type [" << new_edge.type().c_str() << "] couldn't be inserted");
	}
}

template <> 
void dsrAgent::modify_node_attributes<sensor_msgs::msg::BatteryState>(
	std::optional<DSR::Node> &node, const sensor_msgs::msg::BatteryState &msg){
	// Modify the attributes of the node
	G_->add_or_modify_attrib_local<battery_voltage_att>(node.value(), msg.voltage);
	G_->add_or_modify_attrib_local<battery_temperature_att>(node.value(), msg.temperature);
	G_->add_or_modify_attrib_local<battery_current_att>(node.value(), msg.current);
	G_->add_or_modify_attrib_local<battery_charge_att>(node.value(), msg.charge);
	G_->add_or_modify_attrib_local<battery_capacity_att>(node.value(), msg.capacity);
	G_->add_or_modify_attrib_local<battery_design_capacity_att>(node.value(), msg.design_capacity);
	G_->add_or_modify_attrib_local<battery_percentage_att>(node.value(), msg.percentage);
	G_->add_or_modify_attrib_local<battery_power_supply_status_att>(node.value(), static_cast<int>(msg.power_supply_status));
	G_->add_or_modify_attrib_local<battery_power_supply_health_att>(node.value(), static_cast<int>(msg.power_supply_health));
	G_->add_or_modify_attrib_local<battery_power_supply_technology_att>(node.value(), static_cast<int>(msg.power_supply_technology));
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
void dsrAgent::modify_node_attributes<sensor_msgs::msg::Image>(
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
	RCLCPP_DEBUG(this->get_logger(), "Update [%s] node with attributes: ", node.value().name().c_str());
	for (auto &[key, value] : node.value().attrs()){
		RCLCPP_DEBUG(this->get_logger(), "Attribute [%s] = [%s]", key.c_str(), value.value());
	}
}

template <> 
void dsrAgent::modify_node_attributes<sensor_msgs::msg::LaserScan>(
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
	RCLCPP_DEBUG(this->get_logger(), "Update [%s] node with attributes: ", node.value().name().c_str());
	for (auto &[key, value] : node.value().attrs()){
		RCLCPP_DEBUG(this->get_logger(), "Attribute [%s] = [%s]", key.c_str(), value.value());
	}
}

template <typename ROS_TYPE, typename NODE_TYPE, typename EDGE_TYPE> 
void dsrAgent::deserialize_and_update_attributes(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
										const std::string &node_name, const std::string &parent_name){
	// Deserialize a message to ROS_TYPE
	ROS_TYPE ros_msg;
	auto serializer = rclcpp::Serialization<ROS_TYPE>();
	serializer.deserialize_message(msg.get(), &ros_msg);

	// Get the node from the graph, modify its attributes or create it if it does not exist
	if (auto dsr_node = G_->get_node(node_name); dsr_node.has_value()){
		modify_node_attributes<ROS_TYPE>(dsr_node, ros_msg);
		G_->update_node(dsr_node.value());
	}else{
		auto new_id = create_and_insert_node<NODE_TYPE>(node_name);
		if (!parent_name.empty()){
			if (auto parent_node = G_->get_node(parent_name); parent_node.has_value()){
				create_and_insert_edge<EDGE_TYPE>(parent_node.value().id(), new_id.value());
			}
		}
	}
}

void dsrAgent::serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg){
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
		RCLCPP_WARN_ONCE(this->get_logger(), "Received message of type [%s]. Unknown for the DSR.", topic_type.c_str());
	}
}

void dsrAgent::node_updated(std::uint64_t id, const std::string &type){
	if (type == "battery"){
		if (auto node = G_->get_node(id); node.has_value()){
			auto voltage = G_->get_attrib_by_name<battery_voltage_att>(node.value());
			if (voltage.has_value()){
				RCLCPP_INFO(this->get_logger(), "Battery voltage is [%f]", voltage.value());
			}
		}
	}
}

void dsrAgent::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){

}

void dsrAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){

}

void dsrAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){

}

void dsrAgent::node_deleted(std::uint64_t id){

}

void dsrAgent::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){

}