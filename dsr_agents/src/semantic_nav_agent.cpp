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
				generate_goal(get_random_goal_node.value().id(), zone.value());
			}else{
				RCLCPP_WARN(this->get_logger(), "Goal or zone not found in the move node");
			}
		}
	}
	// Check if there an object in the world: object ---(in)--> world
	else if (type == "in"){
		auto object_node = G_->get_node(from);
		auto world_node = G_->get_node(to);
		if (world_node.has_value() && object_node.has_value()){
			// Get the pose of teh object
			auto pose_x = G_->get_attrib_by_name<pose_x_att>(object_node.value());
			auto pose_y = G_->get_attrib_by_name<pose_y_att>(object_node.value());
			auto pose_angle = G_->get_attrib_by_name<pose_angle_att>(object_node.value());
			if (pose_x.has_value() && pose_y.has_value() && pose_angle.has_value()){
				// Get the zone of the object and change its edge
				geometry_msgs::msg::Point point;
				point.x = pose_x.value();
				point.y = pose_y.value();
				get_zone(object_node.value().id(), point);
			}else{
				RCLCPP_WARN(this->get_logger(), "The object doesn't have a pose");
			}
		}
	}
}

void SemanticNavigationAgent::generate_goal(uint64_t node_id, std::string room_name, int n_goals){
	// Create the client for the semantic navigation service
	goals_generator_client_ = this->create_client<GenerateRandomGoals>("generate_random_goals");
	while (!goals_generator_client_->wait_for_service(std::chrono::seconds(5))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Pick a random room if the room_name is 'all'
	std::string random_room_name = room_name;
	if (room_name == "all"){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> distr(0, zones_.size() - 1);
		random_room_name = zones_[distr(gen)];
	}
	RCLCPP_INFO(this->get_logger(), "... to the zone: %s", random_room_name.c_str());

	// Send the request and wait for the response
	auto request = std::make_shared<GenerateRandomGoals::Request>();
	request->n = n_goals;
	request->region_name = random_room_name;
	request->orientation = GenerateRandomGoals::Request::INSIDE;
	request->border = 0.1;

	// Send the request to get the goal
	auto result = goals_generator_client_->async_send_request(request, 
		[this, &node_id](rclcpp::Client<GenerateRandomGoals>::SharedFuture future){
			if (future.get()->goals.poses.size() > 0){
				auto goal = future.get()->goals.poses[0];
				RCLCPP_INFO(this->get_logger(), "Goal generated (%f, %f)", 
					goal.position.x, goal.position.y);
				// Update the goal node in the DSR graph
				if (auto node = G_->get_node(node_id); node.has_value()){
					G_->add_or_modify_attrib_local<goal_x_att>(node.value(), static_cast<float>(goal.position.x));
					G_->add_or_modify_attrib_local<goal_y_att>(node.value(), static_cast<float>(goal.position.y));
					G_->add_or_modify_attrib_local<goal_angle_att>(node.value(), 
						static_cast<float>(tf2::getYaw(goal.orientation)));
					G_->update_node(node.value());
					// Replace the 'wants_to' edge with a 'finished' edge between robot and get_random_goal
					replace_edge<finished_edge_type>(source_, "get_random_goal", "wants_to");
				}
			}else{
				RCLCPP_ERROR(this->get_logger(), "Couldn't send the goal.");
			}
	});
}

void SemanticNavigationAgent::get_zones(){
	// Create the client for the semantic navigation service
	semantic_regions_client_ = this->create_client<ListAllRegions>("list_all_regions");
	while (!semantic_regions_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Send the request to get the list of the zones
	auto request = std::make_shared<ListAllRegions::Request>();
	auto result = semantic_regions_client_->async_send_request(request, 
		[this](rclcpp::Client<ListAllRegions>::SharedFuture future){
			zones_ = future.get()->region_names;

			// Add the list of the zones to the DSR graph
			if (auto world_node = G_->get_node("world"); world_node.has_value()){
				// Add the zones as attributes to the world node
				if (G_->get_priority(world_node.value()) == 0){
					std::vector<std::string> zones_expanded(zones_);
					zones_expanded.push_back("all");
					zones_expanded.push_back("dock");
					std::string zones_joined = boost::algorithm::join(zones_expanded, ",");
					G_->add_or_modify_attrib_local<zones_att>(world_node.value(), zones_joined);
					G_->update_node(world_node.value());
				}
				// Also add the zones as nodes
				for (auto zone : zones_){
					add_node_with_edge<room_node_type, in_edge_type>(zone, "world");
				}
			}
	});
}

void SemanticNavigationAgent::get_zone(uint64_t node_id, const geometry_msgs::msg::Point & point){
	// Create the client for the semantic navigation service
	region_name_client_ = this->create_client<GetRegionName>("get_region_name");
	while (!region_name_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
	}

	// Send the request to get the region name
	auto request = std::make_shared<GetRegionName::Request>();
	request->position = point;

	auto result = region_name_client_->async_send_request(request, 
		[this, &node_id](rclcpp::Client<GetRegionName>::SharedFuture future){
			if (future.get()->region_name != GetRegionName::Response::UNKNOWN){
				auto region_name = future.get()->region_name;
				RCLCPP_INFO(this->get_logger(), "The object is in the zone: %s", region_name.c_str());
				// Change the edge from the object to the world to the object to the zone
				auto object_node = G_->get_node(node_id);
				delete_edge(object_node.value().name(), "world", "in");
				add_edge<in_edge_type>(object_node.value().name(), region_name);
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