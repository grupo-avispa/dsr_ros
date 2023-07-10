/*
 * NAVIGATION AGENT ROS NODE
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// DSR
#include "dsr_agent/ros_to_dsr_types.hpp"
#include "dsr_agent/nav_agent.hpp"

/* Initialize the publishers and subscribers */
navigationAgent::navigationAgent(): AgentNode("navigation_agent"){
	// Get ROS parameters
	get_params();

	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &navigationAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &navigationAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &navigationAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &navigationAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &navigationAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &navigationAgent::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Add the navigation node to the DSR graph
	add_node<navigation_node_type, stopped_edge_type>("navigation", "robot");
}

/* Initialize ROS parameters */
void navigationAgent::get_params(){
	// ROS parameters
	// DSR parameters
	nav2_util::declare_parameter_if_not_declared(this, "dsr_node_name", rclcpp::ParameterValue(""), 
		rcl_interfaces::msg::ParameterDescriptor()
			.set__description("The name of the node in the DSR graph"));
	this->get_parameter("dsr_node_name", dsr_node_name_);
	RCLCPP_INFO(this->get_logger(), 
		"The parameter dsr_node is set to: [%s]", dsr_node_name_.c_str());
}

void navigationAgent::node_updated(std::uint64_t id, const std::string &type){
}

void navigationAgent::node_attributes_updated(uint64_t id, const std::vector<std::string>& att_names){
}

void navigationAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){
	RCLCPP_INFO(this->get_logger(), "Edge updated: %s", type.c_str());
	// Check if the planner wants to start the navigation
	if (type == "start"){
		RCLCPP_INFO(this->get_logger(), "The type is start");
		if (auto planner_node = G_->get_node("planner"); 
				planner_node.has_value() && from == planner_node.value().id() ){
			RCLCPP_INFO(this->get_logger(), "The node is planner");
			if (auto move_node = G_->get_node("move"); 
					move_node.has_value() && from == move_node.value().id() ){
				RCLCPP_INFO(this->get_logger(), "Navigation started");
				// Replace the 'start' edge with a 'is_performing' edge between planner and move
				replace_edge<is_performing_edge_type>(from, to, type);

				auto robot_node = G_->get_node("robot");
				auto navigation_node = G_->get_node("navigation");
				auto stopped_edge = G_->get_edge(
					robot_node.value().name(), navigation_node.value().name(), "stopped");
				auto is_performing_edge = G_->get_edge(
					planner_node.value().name(), move_node.value().name(),"is_performing");
				// Replace the 'stopped' edge with a 'navigating' edge between navigation and robot
				if (stopped_edge.has_value() && is_performing_edge.has_value()){
					if (replace_edge<navigating_edge_type>(
							robot_node.value().id(), navigation_node.value().id(), "stopped")){
						// Get the goal from the move node
						geometry_msgs::msg::Pose goal_pose;
						goal_pose.position.x = G_->get_attrib_by_name<goal_x_att>(
							move_node.value()).value();
						goal_pose.position.y = G_->get_attrib_by_name<goal_y_att>(
							move_node.value()).value();
						goal_pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 
							G_->get_attrib_by_name<goal_angle_att>(move_node.value()).value()));
						// Send the robot to the goal
						send_to_goal(goal_pose);
						// Update the navigation node with the goal
						G_->add_or_modify_attrib_local<goal_x_att>(
							navigation_node.value(), static_cast<float>(goal_pose.position.x));
						G_->add_or_modify_attrib_local<goal_y_att>(
							navigation_node.value(), static_cast<float>(goal_pose.position.y));
						G_->add_or_modify_attrib_local<goal_angle_att>(navigation_node.value(), 
							static_cast<float>(tf2::getYaw(goal_pose.orientation)));
						G_->update_node(navigation_node.value());
						RCLCPP_INFO(this->get_logger(), "Navigation started with goal [%f, %f]", 
							goal_pose.position.x, goal_pose.position.y);
					}
				}
			}
		}
	}

	// Check if the planner wants to abort the navigation
	if (type == "abort"){
		if (auto planner_node = G_->get_node("planner"); 
				planner_node.has_value() && from == planner_node.value().id() ){
			if (auto navigation_node = G_->get_node("navigation"); 
					navigation_node.has_value() && from == navigation_node.value().id() ){
				// Replace the 'abort' edge with a 'aborting' edge between planner and navigation
				if (replace_edge<aborting_edge_type>(from, to, type)){
					cancel_goal();
				}
			}
		}
	}
}

void navigationAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){

}

void navigationAgent::node_deleted(std::uint64_t id){

}

void navigationAgent::edge_deleted(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){

}

void navigationAgent::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle){

}

void navigationAgent::feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
	const std::shared_ptr<const NavigateToPose::Feedback> feedback){
	
	// Set the current pose of the robot
	if (auto robot_node = G_->get_node("robot"); robot_node.has_value()){
		G_->add_or_modify_attrib_local<pose_x_att>(robot_node.value(), 
			static_cast<float>(feedback->current_pose.pose.position.x));
		G_->add_or_modify_attrib_local<pose_y_att>(robot_node.value(), 
			static_cast<float>(feedback->current_pose.pose.position.y));
		G_->add_or_modify_attrib_local<pose_angle_att>(robot_node.value(), 
			static_cast<float>(tf2::getYaw(feedback->current_pose.pose.orientation)));
		G_->update_node(robot_node.value());
	}
}

void navigationAgent::result_callback(const GoalHandleNavigateToPose::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
			auto robot_node = G_->get_node("robot");
			auto navigation_node = G_->get_node("navigation");
			if (auto navigating_edge = G_->get_edge(robot_node.value().name(), 
					navigation_node.value().name(), "navigating"); navigating_edge.has_value()){
				if (replace_edge<stopped_edge_type>(
						robot_node.value().id(), navigation_node.value().id(), "navigating")){
					RCLCPP_INFO(this->get_logger(), "Goal was reached");
				}
			}
			// Replace the 'is_performing' edge with a 'finished' edge between planner and move
			auto planner_node = G_->get_node("planner");
			auto move_node = G_->get_node("move");
			if (auto is_performing_edge = G_->get_edge(planner_node.value().name(), 
					move_node.value().name(),"is_performing"); is_performing_edge.has_value()){
				replace_edge<finished_edge_type>(
					planner_node.value().id(), move_node.value().id(), "is_performing");
			}
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Goal was failed");
			break;
		}
		case rclcpp_action::ResultCode::CANCELED:{
			// Delete 'aborting' edge between planner and navigation
			auto planner_node = G_->get_node("planner");
			auto navigation_node = G_->get_node("navigation");
			if (auto aborting_edge = G_->get_edge(planner_node.value().name(), 
					navigation_node.value().name(), "aborting"); aborting_edge.has_value()){
				G_->delete_edge(
					planner_node.value().name(), navigation_node.value().name(), "aborting");
			}
			// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
			auto robot_node = G_->get_node("robot");
			if (auto navigating_edge = G_->get_edge(robot_node.value().name(), 
					navigation_node.value().name(), "navigating"); navigating_edge.has_value()){
				if (replace_edge<stopped_edge_type>(
						robot_node.value().id(), navigation_node.value().id(), "navigating")){
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
				}
			}
			break;
		}
		default:{
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}
}

void navigationAgent::send_to_goal(geometry_msgs::msg::Pose goal_pose){
	navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
		this->shared_from_this(), "navigate_to_pose");

	if (!this->navigation_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_ERROR(this->get_logger(), "Navigation server not available after waiting");
		return;
	}

	// Check if the navigator is already working towards a goal
	if (goal_handle_ 
		&& (goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED 
		|| goal_handle_->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING))
	{
		RCLCPP_ERROR(this->get_logger(),
			"Navigator already working towards a goal. You need to cancel or wait for the "
			"previous goal.");
		return;
	}

	// Send the goal
	auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
	goal_msg.pose.header.stamp = this->now();
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.pose = goal_pose;

	auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_options.feedback_callback = 
		std::bind(&navigationAgent::feedback_callback, this, 
			std::placeholders::_1, std::placeholders::_2);
	send_options.result_callback =
		std::bind(&navigationAgent::result_callback, this, std::placeholders::_1);

	auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_options);
	if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
		rclcpp::FutureReturnCode::SUCCESS){
		RCLCPP_ERROR(this->get_logger(), "Send goal failed");
		return;
	}

	goal_handle_ = goal_handle_future.get();
	if (!goal_handle_) {
		RCLCPP_ERROR(this->get_logger(), "Navigation server rejected request.");
		return;
	}
	result_future_ = navigation_client_->async_get_result(goal_handle_);
}

void navigationAgent::cancel_goal(){
	if (!goal_handle_ 
		|| (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING 
			&& 	goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED)){
		RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
		return;
	}
	const auto status =
		navigation_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
	if (status != std::future_status::ready) {
		RCLCPP_ERROR(this->get_logger(), "Timed out waiting for navigation goal to cancel.");
	}
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<navigationAgent>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}