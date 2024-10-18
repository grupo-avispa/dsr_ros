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
#include "dsr_agents/tf_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"

TEST_F(DsrUtilTest, tfAgentIntegration) {
  rclcpp::init(0, nullptr);
  // Create the transform publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("tf_publisher");
  pub_node->configure();
  // Create a publisher for the transform
  auto tf_pub = pub_node->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);
  ASSERT_EQ(tf_pub->get_subscription_count(), 0);
  EXPECT_FALSE(tf_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(tf_pub->is_activated());
  tf_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create and configure the tf_agent node
  auto agent_node = std::make_shared<dsr_agents::TFAgent>();
  // Set some parameters
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Create a tf message
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped trf;
  trf.header.frame_id = "map";
  trf.child_frame_id = "base_link";
  tf_msg.transforms.push_back(trf);

  // Publish the message
  tf_pub->publish(tf_msg);

  // Spin the tf_agent node
  rclcpp::spin_some(agent_node->get_node_base_interface());

  // Check the results: now, the tf should have a subscription
  EXPECT_EQ(tf_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  agent_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}
