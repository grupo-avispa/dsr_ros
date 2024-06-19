/*
 * SEMANTIC NAVIGATION AGENT ROS NODE
 *
 * Copyright (c) 2024 Alberto José Tudela Roldán <ajtudela@gmail.com>
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
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/semantic_nav_agent.hpp"

/* Initialize the publishers and subscribers */
SemanticNavigationAgent::SemanticNavigationAgent(): AgentNode("semantic_navigation_agent"){
	// Add connection signals
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_signal, this, &SemanticNavigationAgent::node_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_node_attr_signal, this, &SemanticNavigationAgent::node_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_signal, this, &SemanticNavigationAgent::edge_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::update_edge_attr_signal, this, &SemanticNavigationAgent::edge_attributes_updated);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_edge_signal, this, &SemanticNavigationAgent::edge_deleted);
	QObject::connect(G_.get(), 
		&DSR::DSRGraph::del_node_signal, this, &SemanticNavigationAgent::node_deleted);

	// Wait until the DSR graph is ready
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Get the list of the zones
	get_zones();
}

void SemanticNavigationAgent::edge_updated(std::uint64_t from, std::uint64_t to,  const std::string &type){
	// Check if the robot wants to abort or cancel the action: robot ---(abort)--> get_random_goal
	if (type == "abort" || type == "cancel"){
		auto robot_node = G_->get_node(from);
		auto get_random_goal_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& get_random_goal_node.has_value() && get_random_goal_node.value().type() == "get_random_goal"){
			RCLCPP_INFO(this->get_logger(), "Starting to %s the get_random_goal", type.c_str());
			// Remove the get_random_goal node from the DSR graph
			if (G_->delete_node(get_random_goal_node.value())){
				RCLCPP_INFO(this->get_logger(), "Get random goal %sed", type.c_str());
			}
		}
	}
	// Check if the robot wants to start the action: robot ---(wants_to)--> get_random_goal
	else if (type == "wants_to"){
		auto robot_node = G_->get_node(from);
		auto get_random_goal_node = G_->get_node(to);
		if (robot_node.has_value() &&  robot_node.value().name() == source_
			&& get_random_goal_node.has_value() && get_random_goal_node.value().type() == "get_random_goal"){
			RCLCPP_INFO(this->get_logger(), "Getting the goal...");
			// Get the attributes from the node
			auto zone = G_->get_attrib_by_name<zone_att>(get_random_goal_node.value());
			if (zone.has_value()){
				// Generate the goal
				geometry_msgs::msg::Pose goal = generate_goal(zone.value());
				// Add the goal to the DSR graph
				G_->add_or_modify_attrib_local<goal_x_att>(
					get_random_goal_node.value(), static_cast<float>(goal.position.x));
				G_->add_or_modify_attrib_local<goal_y_att>(
					get_random_goal_node.value(), static_cast<float>(goal.position.y));
				G_->add_or_modify_attrib_local<goal_angle_att>(
					get_random_goal_node.value(), static_cast<float>(tf2::getYaw(goal.orientation)));
				G_->update_node(get_random_goal_node.value());
			}else{
				RCLCPP_WARN(this->get_logger(), "Goal or zone not found in the move node");
			}
		}
	}
}

geometry_msgs::msg::Pose SemanticNavigationAgent::generate_goal(std::string room_name, int n_goals){
	// Create the clients for the semantic navigation services
	goals_generator_client_ = this->create_client<SemanticGoals>("generate_random_goals");
	while (!goals_generator_client_->wait_for_service(std::chrono::seconds(5))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return geometry_msgs::msg::Pose();
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
	request->region_name = room_name;
	request->orientation = SemanticGoals::Request::INSIDE;
	request->border = 0.1;
	auto result = goals_generator_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticGoals>::SharedFuture result){
			if (result.get()->goals.poses.size() > 0){
				// Send goals to the navigation stack
				return result.get()->goals.poses[0];
			}else{
				RCLCPP_ERROR(this->get_logger(), "Couldn't send the goal.");
			}
	});

	return geometry_msgs::msg::Pose();
}

void SemanticNavigationAgent::get_zones(){
	// Create the clients for the semantic navigation services
	semantic_regions_client_ = this->create_client<SemanticRegions>("list_all_regions");
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
			zones_ = result.get()->region_names;

			// Add the list of the zones to the DSR graph
			if (auto world_node = G_->get_node("world"); world_node.has_value()){
				if (G_->get_priority(world_node.value()) == 0){
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

int main(int argc, char** argv){
	QCoreApplication app(argc, argv);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<SemanticNavigationAgent>();

	QtExecutor exe;
	exe.add_node(node);
	exe.start();

	auto res = app.exec();
	rclcpp::shutdown();
	return res;
}