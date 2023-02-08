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

// QT
#include <QWidget>

// ROS
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "dsr_agent/dsr_agent.hpp"
#include "dsr_agent/ros_attr_name.hpp"

/* Initialize the publishers and subscribers */
dsrAgent::dsrAgent(): Node("dsr_agent"){
	// Get ROS parameters
	get_params();

	// Create graph
	//G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "");

	//! This os only for testing
	G_ = std::make_shared<DSR::DSRGraph>(0, agent_name_, agent_id_, "/home/alberto/initial_dsr.json");
	RCLCPP_INFO(this->get_logger(), "Graph loaded");
	auto world_node = G_->get_node("world");
	if(!world_node.has_value()){
		RCLCPP_ERROR(this->get_logger(), "World node not found");
		auto world_node = DSR::Node::create<world_node_type>("world");
		auto id = G_->insert_node(world_node);
		RCLCPP_INFO(this->get_logger(), "World node created with id: %d", id.value());
	}else{
		G_->update_node(world_node.value());
	}

	// Subscriber to the topic with a generic subscription
	auto data = rclcpp::Node::get_topic_names_and_types();
	for (auto type : data[ros_topic_]){
		generic_sub_ = create_generic_subscription(
						ros_topic_, 
						type, 
						rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
						std::bind(&dsrAgent::serial_callback, this, std::placeholders::_1)
						);
	}
}

dsrAgent::~dsrAgent() {
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
}

void dsrAgent::serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg){
	// In order to deserialize the message we have to manually create a ROS2
	// message in which we want to convert the serialized data.
	auto data = rclcpp::Node::get_topic_names_and_types();
	const std::string topic_type = data[ros_topic_][0];
	RCLCPP_INFO_ONCE(this->get_logger(), "Subscribed to topic [%s] of type [%s]", 
						ros_topic_.c_str(), topic_type.c_str());

	if (topic_type == "sensor_msgs/msg/BatteryState"){
		// Create a ROS2 message of type BatteryState
		sensor_msgs::msg::BatteryState battery_msg;
		auto serializer = rclcpp::Serialization<sensor_msgs::msg::BatteryState>();
		serializer.deserialize_message(msg.get(), &battery_msg);
		// Get the node from the graph
		if (auto dsr_node = G_->get_node(dsr_node_name_); dsr_node.has_value()){
			modify_battery_attributes_and_update(dsr_node, battery_msg);
			//modify_node_attributes_and_update<sensor_msgs::msg::BatteryState>(dsr_node, battery_msg);
		}else{
			create_and_insert_node<battery_node_type>(dsr_node_name_);
		}

		// Using template (Optional)
		//deserialize_and_update<sensor_msgs::msg::BatteryState, battery_node_type>(msg, dsr_node_name_);
	}else if (topic_type == "std_msgs/msg/String"){
		// Create a ROS2 message of type String
		// TODO: Publish to DSR
	}else{
		RCLCPP_WARN_ONCE(this->get_logger(), "Received message of type [%s]. Unknown for the DSR.", topic_type.c_str());
	}
}
template <typename R, typename D> 
void dsrAgent::deserialize_and_update(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
										const std::string &node_name){
	// Create a ROS2 message of type R
	R ros_msg;
	auto serializer = rclcpp::Serialization<R>();
	serializer.deserialize_message(msg.get(), &ros_msg);
	// Get the node from the graph
	if (auto dsr_node = G_->get_node(node_name); dsr_node.has_value()){
		modify_battery_attributes_and_update(dsr_node, ros_msg);
		//modify_node_attributes_and_update<sensor_msgs::msg::BatteryState>(dsr_node, battery_msg);
	}else{
		create_and_insert_node<D>(node_name);
	}
}

template <typename T> 
void dsrAgent::create_and_insert_node(const std::string &name){
	RCLCPP_ERROR(this->get_logger(), "%s node not found", name.c_str());
	auto new_dsr_node = DSR::Node::create<T>(name);
	auto id = G_->insert_node(new_dsr_node);
	RCLCPP_INFO(this->get_logger(), "%s node created with id [%d]", id.value());
}

void dsrAgent::modify_battery_attributes_and_update(std::optional<DSR::Node> node, 
												const sensor_msgs::msg::BatteryState &msg){
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
	// Update the node in the graph to set the modified attributes available
	G_->update_node(node.value());
	RCLCPP_INFO(this->get_logger(), "%s node updated", node.value().name());
}

template <> 
void dsrAgent::modify_node_attributes_and_update<sensor_msgs::msg::BatteryState>(
	std::optional<DSR::Node> node, const sensor_msgs::msg::BatteryState &msg){

}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<dsrAgent>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
