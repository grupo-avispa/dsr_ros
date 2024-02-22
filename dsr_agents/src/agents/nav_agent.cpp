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
#include <random>

// BOOST
#include <boost/algorithm/string/join.hpp>

// ROS
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// DSR
#include "dsr_agents/qt_executor.hpp"
#include "dsr_agents/agents/nav_agent.hpp"

/* Initialize the publishers and subscribers */
NavigationAgent::NavigationAgent(): AgentNode("navigation_agent"), current_zone_(""){
	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &NavigationAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &NavigationAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &NavigationAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &NavigationAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &NavigationAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &NavigationAgent::node_deleted);

	// Initialize transform buffer and listener
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Add the 'navigation' node with and edge 'stopped' hanging from the 'robot' node into the DSR graph
	if (auto nav_node = G_->get_node("navigation"); !nav_node.has_value()){
		add_node_with_edge<navigation_node_type, stopped_edge_type>("navigation", source_);
	}

	// Get the list of the zones
	get_zones();
}

void NavigationAgent::node_updated(std::uint64_t id, const std::string &type){
	// Update the current robot pose into the DSR graph when the navigation node is updated
	if (auto node = G_->get_node(id); node.has_value() && node.value().type() == "navigation"){
		// Get the current pose of the robot
		geometry_msgs::msg::PoseStamped robot_pose;
		if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, "map", "base_link", 2.0)){
			return;
		}
		// Update the pose of the robot into the DSR graph
		update_robot_pose_in_dsr(robot_pose.pose);
	}
}

void NavigationAgent::node_attributes_updated(uint64_t id, 
	const std::vector<std::string>& att_names){
}

void NavigationAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){
	// Check if the robot wants to abort or cancel the navigation: robot ---(abort)--> move
	if (type == "abort" || type == "cancel"){
		RCLCPP_INFO(this->get_logger(), "Starting to %s the navigation", type.c_str());
		auto robot_node = G_->get_node(from);
		auto move_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& move_node.has_value() && move_node.value().type() == "move"){
			// Remove the move node from the DSR graph
			if (G_->delete_node(move_node.value())){
				cancel_goal();
				RCLCPP_INFO(this->get_logger(), "Navigation %sed", type.c_str());
			}
		}
	}
	// Check if the robot wants to start the navigation: robot ---(wants_to)--> move
	else if (type == "wants_to"){
		RCLCPP_INFO(this->get_logger(), "Starting the navigation");
		auto robot_node = G_->get_node(from);
		auto move_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& move_node.has_value() && move_node.value().type() == "move"){
			// Get the attributes from the move node
			auto goal_x = G_->get_attrib_by_name<goal_x_att>(move_node.value());
			auto goal_y = G_->get_attrib_by_name<goal_y_att>(move_node.value());
			auto goal_angle = G_->get_attrib_by_name<goal_angle_att>(move_node.value());
			auto zone = G_->get_attrib_by_name<zone_att>(move_node.value());
			// Check if the goal is a point or a room and send the robot
			if (goal_x.has_value() && goal_y.has_value() && goal_angle.has_value()){
				send_to_goal(geometry_msgs::build<geometry_msgs::msg::Pose>()
					.position(geometry_msgs::build<geometry_msgs::msg::Point>()
						.x(goal_x.value()).y(goal_y.value()).z(0))
					.orientation(tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 
						goal_angle.value()))));
				RCLCPP_INFO(this->get_logger(), "Navigation started to goal [%f, %f]", 
					goal_x.value(), goal_y.value());
			}else if (zone.has_value()){
				// Check if the zone is dock and start docking or send the robot to the room
				if (zone.value() == "dock"){
					start_docking();
				}else{
					send_to_room(zone.value());
				}
				RCLCPP_INFO(this->get_logger(), "Navigation started to room [%s]", 
					zone.value().c_str());
				// Update the zone into the DSR graph
				if (auto nav_node = G_->get_node("navigation"); nav_node.has_value()){
					if (get_priority(nav_node.value()) == 0){
						G_->add_or_modify_attrib_local<zone_att>(nav_node.value(), zone.value());
						G_->update_node(nav_node.value());
					}
				}
			}else{
				RCLCPP_WARN(this->get_logger(), "Goal or zone not found in the move node");
			}
		}
	}
}

void NavigationAgent::edge_attributes_updated(std::uint64_t from, std::uint64_t to, 
	const std::string &type, const std::vector<std::string>& att_names){
}

void NavigationAgent::node_deleted(std::uint64_t id){
}

void NavigationAgent::edge_deleted(std::uint64_t from, std::uint64_t to, 
	const std::string &edge_tag){
}

void NavigationAgent::nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr 
	& goal_handle){
	goal_handle_ = goal_handle;
	if (!goal_handle_){
		RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server");
	}else{
		// Replace the 'wants_to' edge with a 'is_performing' edge between robot and move
		if (replace_edge<is_performing_edge_type>(source_, "move", "wants_to")){
			RCLCPP_INFO(this->get_logger(), 
				"Navigation goal accepted by server, waiting for result");
		}
	}
}

void NavigationAgent::nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr, 
	const std::shared_ptr<const NavigateToPose::Feedback> feedback){

	// Replace the 'stopped' edge with a 'navigating' edge between robot and navigation
	auto stopped_edge = G_->get_edge(source_, "navigation", "stopped");
	if (stopped_edge.has_value()){
		replace_edge<navigating_edge_type>(source_, "navigation", "stopped");
	}

	// Set the current pose of the robot into the DSR graph
	update_robot_pose_in_dsr(feedback->current_pose.pose);
}

void NavigationAgent::nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result){
	// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
	if (replace_edge<stopped_edge_type>(source_, "navigation", "navigating")){
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				// Replace the 'is_performing' edge with a 'finished' edge between robot and move
				if (replace_edge<finished_edge_type>(source_, "move", "is_performing")){
					RCLCPP_INFO(this->get_logger(), "Goal reached");
				}
				break;
			case rclcpp_action::ResultCode::ABORTED:
				// Replace the 'is_performing' edge with a 'failed' edge between robot and move
				if (replace_edge<failed_edge_type>(source_, "move", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Goal aborted");
				}
				break;
			case rclcpp_action::ResultCode::CANCELED:
				// Replace the 'is_performing' edge with a 'canceled' edge between robot and move
				if (replace_edge<cancel_edge_type>(source_, "move", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Goal canceled");
				}
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
		}
	}
}

void NavigationAgent::dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(this->get_logger(), "Docking goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Docking goal accepted by server, waiting for result");
	}
}

void NavigationAgent::dock_feedback_callback(GoalHandleDock::SharedPtr, 
	const std::shared_ptr<const Dock::Feedback> feedback){
}

void NavigationAgent::dock_result_callback(const GoalHandleDock::WrappedResult & result){
	// Replace the 'navigating' edge with a 'stopped' edge between robot and navigation
	if (replace_edge<stopped_edge_type>(source_, "navigation", "navigating")){
		switch (result.code) {
			case rclcpp_action::ResultCode::SUCCEEDED:
				// Replace the 'is_performing' edge with a 'finished' edge between robot and move
				if (replace_edge<finished_edge_type>(source_, "move", "is_performing")){
					RCLCPP_INFO(this->get_logger(), "Docking succeeded");
				}
				break;
			case rclcpp_action::ResultCode::ABORTED:
				// Replace the 'is_performing' edge with a 'failed' edge between robot and move
				if (replace_edge<failed_edge_type>(source_, "move", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Docking aborted");
				}
				break;
			case rclcpp_action::ResultCode::CANCELED:
				// Replace the 'is_performing' edge with a 'canceled' edge between robot and move
				if (replace_edge<cancel_edge_type>(source_, "move", "is_performing")){
					RCLCPP_ERROR(this->get_logger(), "Docking canceled");
				}
				break;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				break;
		}
	}
}

void NavigationAgent::undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(this->get_logger(), "Undocking goal was rejected by server");
	}else{
		RCLCPP_INFO(this->get_logger(), "Undocking goal accepted by server, waiting for result");
	}
}

void NavigationAgent::undock_feedback_callback(GoalHandleUndock::SharedPtr, 
	const std::shared_ptr<const Undock::Feedback> feedback){
}

void NavigationAgent::undock_result_callback(const GoalHandleUndock::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:{
			RCLCPP_INFO(this->get_logger(), "Undocking succeeded");
			break;
		}
		case rclcpp_action::ResultCode::ABORTED:{
			RCLCPP_ERROR(this->get_logger(), "Undocking aborted");
			break;
		}
		case rclcpp_action::ResultCode::CANCELED:{
			RCLCPP_ERROR(this->get_logger(), "Undocking canceled");
			break;
		}
		default:{
			RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			break;
		}
	}
}

void NavigationAgent::send_to_room(std::string room_name, int n_goals){
	// Create the clients for the semantic navigation services
	goals_generator_client_ = this->create_client<SemanticGoals>("semantic_goals");
	while (!goals_generator_client_->wait_for_service(std::chrono::seconds(5))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Pick a random room if the room_name is 'all'
	if (room_name == "all"){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> distr(0, zones_.size() - 1);
		room_name = zones_[distr(gen)];
	}

	// Send the request and wait for the response
	geometry_msgs::msg::PoseArray goals;
	auto request = std::make_shared<SemanticGoals::Request>();
	request->n = n_goals;
	request->roi_name = room_name;
	request->direction = semantic_navigation_msgs::srv::SemanticGoals::Request::INSIDE;
	request->border = 0.1;
	auto result = goals_generator_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticGoals>::SharedFuture result){
			if (result.get()->goals.poses.size() > 0){
				// Send goals to the navigation stack
				send_to_goal(result.get()->goals.poses[0]);
			}else{
				RCLCPP_ERROR(this->get_logger(), "Couldn't send the goal.");
			}
	});
}

void NavigationAgent::send_to_goal(geometry_msgs::msg::Pose goal_pose){
	using namespace std::placeholders;
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
		shared_from_this(), "navigate_to_pose");

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
			"Navigator already working towards a goal. Cancelling the previous goal.");
		cancel_goal();
	}

	// Populate a goal message
	auto goal_msg = NavigateToPose::Goal();
	goal_msg.pose.header.stamp = this->now();
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.pose = goal_pose;

	auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_options.goal_response_callback =
		std::bind(&NavigationAgent::nav_goal_response_callback, this, _1);
	send_options.feedback_callback = 
		std::bind(&NavigationAgent::nav_feedback_callback, this, _1, _2);
	send_options.result_callback =
		std::bind(&NavigationAgent::nav_result_callback, this, _1);

	auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_options);
	RCLCPP_INFO(this->get_logger(), "Goal sent");
}

void NavigationAgent::cancel_goal(){
	if (!goal_handle_ 
		|| (goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_EXECUTING 
			&& goal_handle_->get_status() != rclcpp_action::GoalStatus::STATUS_ACCEPTED)){
		RCLCPP_WARN(this->get_logger(), "Cancel called with no active goal.");
		return;
	}
	try{
		const auto future_status =
			navigation_client_->async_cancel_goal(goal_handle_).wait_for(std::chrono::seconds(5));
		if (future_status != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "Timed out waiting for navigation goal to cancel.");
		}
	} catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
		RCLCPP_ERROR(this->get_logger(), "Failed to cancel goal: Unknown goal handle");
	}
}

void NavigationAgent::get_zones(){
	// Create the clients for the semantic navigation services
	semantic_regions_client_ = this->create_client<SemanticRegions>("semantic_regions");
	while (!semantic_regions_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Send the request to get the list of the zones
	auto request = std::make_shared<SemanticRegions::Request>();
	auto result = semantic_regions_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticRegions>::SharedFuture result){
			zones_ = result.get()->regions;

			// Add the list of the zones to the DSR graph
			if (auto world_node = G_->get_node("world"); world_node.has_value()){
				if (get_priority(world_node.value()) == 0){
					std::vector<std::string> zones_expanded(zones_);
					zones_expanded.push_back("all");
					zones_expanded.push_back("dock");
					std::string zones_joined = boost::algorithm::join(zones_expanded, ",");
					G_->add_or_modify_attrib_local<zones_att>(world_node.value(), zones_joined);
					G_->update_node(world_node.value());
				}
			}
	});
}

void NavigationAgent::update_robot_pose_in_dsr(geometry_msgs::msg::Pose pose){
	// Update the robot pose into the DSR graph
	if (auto robot_node = G_->get_node(source_); robot_node.has_value()){
		if (get_priority(robot_node.value()) == 0){
			G_->add_or_modify_attrib_local<pose_x_att>(robot_node.value(), 
				static_cast<float>(pose.position.x));
			G_->add_or_modify_attrib_local<pose_y_att>(robot_node.value(), 
				static_cast<float>(pose.position.y));
			G_->add_or_modify_attrib_local<pose_angle_att>(robot_node.value(), 
				static_cast<float>(tf2::getYaw(pose.orientation)));
			G_->update_node(robot_node.value());
		}
	}
}

void NavigationAgent::start_docking(){
	using namespace std::placeholders;
	dock_client_ = rclcpp_action::create_client<Dock>(shared_from_this(), "dock");

	if (dock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Dock::Goal();
		goal_msg.station_id = -1;
		goal_msg.timeout = 120;

		auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&NavigationAgent::dock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&NavigationAgent::dock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&NavigationAgent::dock_result_callback, this, _1);
		dock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Docking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

void NavigationAgent::start_undocking(){
	undock_client_ = rclcpp_action::create_client<Undock>(shared_from_this(), "undock");
	using namespace std::placeholders;

	if (undock_client_->wait_for_action_server(std::chrono::seconds(1))){
		auto goal_msg = Undock::Goal();
		goal_msg.timeout = 120;

		auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&NavigationAgent::undock_goal_response_callback, this, _1);
		send_goal_options.feedback_callback =
			std::bind(&NavigationAgent::undock_feedback_callback, this, _1, _2);
		send_goal_options.result_callback =
			std::bind(&NavigationAgent::undock_result_callback, this, _1);
		undock_client_->async_send_goal(goal_msg, send_goal_options);
		RCLCPP_INFO(this->get_logger(), "Undocking");
	}else{
		RCLCPP_WARN(this->get_logger(), "Action server not available after waiting");
	}
}

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<NavigationAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}