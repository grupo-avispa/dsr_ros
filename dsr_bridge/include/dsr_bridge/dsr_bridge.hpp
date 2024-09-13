// Copyright (c) 2024 Óscar Pons Fernández
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Jose Miguel Galeas Merchan
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

#ifndef DSR_BRIDGE__DSR_BRIDGE_HPP_
#define DSR_BRIDGE__DSR_BRIDGE_HPP_

// Qt
#include <QObject>

// C++
#include <map>
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs/msg/edge.hpp"
#include "dsr_msgs/msg/node.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class DSRBridge : public dsr_util::AgentNode
{
public:
  DSRBridge();

private:
  rclcpp::Subscription<dsr_msgs::msg::Edge>::SharedPtr edge_from_ros_sub_;
  rclcpp::Subscription<dsr_msgs::msg::Node>::SharedPtr node_from_ros_sub_;
  rclcpp::Publisher<dsr_msgs::msg::Edge>::SharedPtr edge_to_ros_pub_;
  rclcpp::Publisher<dsr_msgs::msg::Node>::SharedPtr node_to_ros_pub_;
  std::string edge_topic_, node_topic_;
  // Struct and vector to store edges when nodes are not created yet
  struct lost_edge
  {
    std::string from;
    std::string to;
    std::string type;
    std::vector<std::string> atts;
    bool operator==(const lost_edge & other_edge) const
    {
      return (from == other_edge.from) && (to == other_edge.to) && (type == other_edge.type);
    }
    lost_edge(
      std::string parent, std::string child, std::string edge_type,
      std::vector<std::string> & attributes)
    : from(parent), to(child), type(edge_type),
      atts(attributes) {}
  };
  std::vector<lost_edge> lost_edges;
  void get_params();

  // ROS callbacks
  void edge_from_ros_callback(const dsr_msgs::msg::Edge::SharedPtr msg);
  void node_from_ros_callback(const dsr_msgs::msg::Node::SharedPtr msg);

  // DSR callbacks
  void node_created(std::uint64_t id, const std::string & type);
  void node_attr_updated(uint64_t id, const std::vector<std::string> & att_names) override;
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override;
  void edge_attr_updated(
    std::uint64_t from, std::uint64_t to,
    const std::string & type, const std::vector<std::string> & att_names) override;
  void node_deleted(const DSR::Node & node);
  void edge_deleted(std::uint64_t from, std::uint64_t to, const std::string & edge_tag) override;

  // Converter functions
  std::optional<DSR::Node> create_dsr_node(std::string name, std::string type);
  std::optional<DSR::Edge> create_dsr_edge(
    std::string from, std::string to, const std::string & type, std::vector<std::string> & atts);
  dsr_msgs::msg::Node create_msg_node(std::string name, std::string type);
  dsr_msgs::msg::Edge create_msg_edge(
    std::uint64_t from, std::uint64_t to, const std::string & type);

  // Helper functions
  template<typename TYPE>
  void modify_attributes(TYPE & elem, std::vector<std::string> & att_str);

  std::string attribute_to_string(const DSR::Attribute & att);
  std::vector<std::string> attributes_to_string(
    const std::map<std::string, DSR::Attribute> & atts);
  template<typename TYPE>
  std::vector<std::string> attributes_updated_to_string(
    TYPE & elem, const std::vector<std::string> & atts);
  DSR::Attribute string_to_attribute(const std::string & att_value, int att_type);
  std::string get_type_from_attribute(const DSR::Attribute & att);
};

#endif  // DSR_BRIDGE__DSR_BRIDGE_HPP_
