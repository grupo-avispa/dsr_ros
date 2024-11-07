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

#include "gtest/gtest.h"
#include "dsr_agents/topic_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

TEST_F(DsrUtilTest, topicAgentIntegration) {
  rclcpp::init(0, nullptr);
  // Create the transform publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("topic_publisher");
  pub_node->configure();
  // Create a publisher for the topic
  auto topic_pub = pub_node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
  ASSERT_EQ(topic_pub->get_subscription_count(), 0);
  EXPECT_FALSE(topic_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(topic_pub->is_activated());
  topic_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create and configure the topic_agent node
  auto agent_node = std::make_shared<dsr_agents::TopicAgent>();
  // Set some parameters
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->declare_parameter("ros_topic", rclcpp::ParameterValue("/scan"));
  agent_node->configure();
  agent_node->activate();

  // Create a message
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.frame_id = "laser_frame";

  // Publish the message
  topic_pub->publish(scan_msg);

  // Spin the tf_agent node
  rclcpp::spin_some(agent_node->get_node_base_interface());

  // Check the results: now, the topic should have a subscription
  EXPECT_EQ(topic_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  agent_node->deactivate();
  agent_node->cleanup();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}
