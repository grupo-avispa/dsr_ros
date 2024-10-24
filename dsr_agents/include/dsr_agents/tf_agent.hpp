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

#ifndef DSR_AGENTS__TF_AGENT_HPP_
#define DSR_AGENTS__TF_AGENT_HPP_


// C++
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

namespace dsr_agents
{

/**
 * @class dsr_agents::TFAgent
 * @brief Agent to receive and send TF messages from ROS 2 to the DSR graph.
 */
class TFAgent : public dsr_util::AgentNode
{
public:
  /**
   * @brief Construct a new TFAgent object.
   *
   * @param options Node options
   */
  explicit TFAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Callback to receive TF messages from ROS 2.
   *
   * @param msg TF message
   */
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  /**
   * @brief Sort the TF messages by parent frame.
   *
   * @param unsorted_trf Unsorted TF message
   * @return tf2_msgs::msg::TFMessage Sorted TF message
   */
  tf2_msgs::msg::TFMessage sort_tf_by_parent_frame(tf2_msgs::msg::TFMessage & unsorted_trf);

  /**
   * @brief Replace the frames with the DSR names.
   * The DSR names are the name of the robot (set by source_) for 'base_link' and 'world' for 'map'.
   *
   * @param sorted_trf Sorted TF message
   */
  void replace_frames_with_dsr_names(tf2_msgs::msg::TFMessage & sorted_trf);

  /**
   * @brief Insert the TF message into the DSR graph.
   *
   * @param sorted_trf Sorted TF message
   */
  void insert_and_update_tf_into_dsr(const tf2_msgs::msg::TFMessage & sorted_trf);

  /**
   * @brief Store the DSR names in a vector.
   *
   * @param sorted_trf Sorted TF message
   * @param std::vector<std::string> Vector with the DSR names
   */
  void store_dsr_names(
    const tf2_msgs::msg::TFMessage & sorted_trf, std::vector<std::string> & dsr_names);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_, tf_static_sub_;

  // List of nodes created by the agent
  std::vector<std::string> dsr_nodes_;
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__TF_AGENT_HPP_
