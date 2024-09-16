// Copyright (c) 2023 Óscar Pons Fernández
// Copyright (c) 2023 Alberto J. Tudela Roldán
// Copyright (c) 2023 Jose Miguel Galeas Merchan
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

#ifndef DSR_AGENTS__WHISPER_AGENT_HPP_
#define DSR_AGENTS__WHISPER_AGENT_HPP_

// C++
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

namespace dsr_agents
{

class WhisperAgent : public dsr_util::AgentNode
{
public:
  WhisperAgent();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr whisper_sub_;
  std::string ros_topic_;

  /// Get ROS params
  void get_params();

  /// Person detection callback
  void whisper_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace dsr_agents

#endif  // DSR_AGENTS__WHISPER_AGENT_HPP_
