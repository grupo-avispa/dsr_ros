// Copyright (c) 2023 Óscar Pons Fernández
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

#ifndef DSR_AGENTS__PERSON_AGENT_HPP_
#define DSR_AGENTS__PERSON_AGENT_HPP_

// Qt
#include <QObject>

// C++
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "person_msgs/msg/person.hpp"
#include "person_msgs/msg/person_array.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_util/agent_node.hpp"

class PersonAgent : public dsr_util::AgentNode
{
public:
  PersonAgent();

private:
  rclcpp::Subscription<person_msgs::msg::PersonArray>::SharedPtr person_sub_;
  std::string ros_topic_;

  /// The buffer of the transformations tree.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /// The listener of the transformations tree.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// Timer to remove people from DSR if they are missed more then 30s
  rclcpp::TimerBase::SharedPtr timer_;

  /// Maximum elapsed time to remove people from DSR
  int timeout_;

  /// Get ROS params
  void get_params();
  /// Person detection callback
  void person_callback(const person_msgs::msg::PersonArray::SharedPtr msg);
  /// Timeout callback for delete people from DSR
  void remove_callback();
};

#endif  // DSR_AGENTS__PERSON_AGENT_HPP_
