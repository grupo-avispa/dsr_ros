// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_
#define DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_

// Qt
#include <QObject>

// C++
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/get_region_name.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class SemanticNavigationAgent : public dsr_util::AgentNode
{
public:
  SemanticNavigationAgent();

private:
  using GenerateRandomGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
  using GetRegionName = semantic_navigation_msgs::srv::GetRegionName;
  using ListAllRegions = semantic_navigation_msgs::srv::ListAllRegions;

  rclcpp::Client<GenerateRandomGoals>::SharedPtr goals_generator_client_;
  rclcpp::Client<GetRegionName>::SharedPtr region_name_client_;
  rclcpp::Client<ListAllRegions>::SharedPtr semantic_regions_client_;
  std::vector<std::string> zones_;

  void generate_goal(uint64_t node_id, std::string room_name, int n_goals = 1);
  void get_zones();
  void get_zone(uint64_t node_id, const geometry_msgs::msg::Point & point);

  // DSR callbacks
  void node_updated(std::uint64_t /*id*/, const std::string & /*type*/) {}
  void node_attr_updated(uint64_t /*id*/, const std::vector<std::string> & /*att_names*/) {}
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type);
  void edge_attr_updated(
    std::uint64_t /*from*/, std::uint64_t /*to*/,
    const std::string & /*type*/, const std::vector<std::string> & /*att_names*/) {}
  void node_deleted(std::uint64_t /*id*/) {}
  void edge_deleted(
    std::uint64_t /*from*/, std::uint64_t /*to*/, const std::string & /*edge_tag*/) {}
};

#endif  // DSR_AGENTS__SEMANTIC_NAV_AGENT_HPP_
