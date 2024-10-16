// Copyright (c) 2023 Alberto J. Tudela Roldán
// Copyright (c) 2023 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef DSR_AGENTS__TOPIC_AGENT_HPP_
#define DSR_AGENTS__TOPIC_AGENT_HPP_

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialization.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

namespace dsr_agents
{

/**
 * @class dsr_agents::TopicAgent
 * @brief Agent to receive and send messages from ROS 2 topics to the DSR graph.
 */
class TopicAgent : public dsr_util::AgentNode
{
public:
  /**
   * @brief Construct a new TopicAgent object.
   *
   * @param options Node options
   */
  explicit TopicAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

private:
  /**
   * @brief Modify the attributes of the node depending on the type of the ROS message
   * and the DSR type.
   *
   * @tparam ROS_TYPE Type of the ROS message
   * @tparam NODE_TYPE Type of the DSR node
   * @param node DSR node
   * @param msg ROS message
   */
  template<typename ROS_TYPE> void modify_attributes(
    std::optional<DSR::Node> & node, const ROS_TYPE & msg);

  /**
   * @brief Deserialize the message and update the attributes in the DSR graph.
   *
   * @tparam ROS_TYPE Type of the ROS message
   * @tparam NODE_TYPE Type of the DSR node
   * @tparam EDGE_TYPE Type of the DSR edge
   * @param msg Serialized message
   * @param node_name Name of the DSR node
   * @param parent_name Name of the parent DSR node
   */
  template<typename ROS_TYPE, typename NODE_TYPE, typename EDGE_TYPE>
  void deserialize_and_update_attributes(
    const std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & node_name, const std::string & parent_name);

  /**
   * @brief Callback to receive messages from ROS 2 topics.
   *
   * @param msg Serialized message
   */
  void serial_callback(const std::shared_ptr<rclcpp::SerializedMessage> msg);

  rclcpp::GenericSubscription::SharedPtr generic_sub_;
  std::string ros_topic_, dsr_node_name_, dsr_parent_node_name_;
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__TOPIC_AGENT_HPP_
