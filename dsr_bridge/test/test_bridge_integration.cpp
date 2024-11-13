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

#include <QCoreApplication>
#include <QtTest/QSignalSpy>
#include <thread>

#include "gtest/gtest.h"
#include "dsr_bridge/dsr_bridge.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"

class DSRBridgeFixture : public dsr_bridge::DSRBridge
{
public:
  DSRBridgeFixture()
  : dsr_bridge::DSRBridge()
  {
  }

  ~DSRBridgeFixture() = default;

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }
};

TEST_F(DsrUtilTest, DSRBridgeIntegrationNode) {
  rclcpp::init(0, nullptr);

  // Create a publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_publisher");
  pub_node->configure();

  // Create a publisher for the node_msgs
  auto msg_pub = pub_node->create_publisher<dsr_msgs::msg::Node>("nodes", 1);
  ASSERT_EQ(msg_pub->get_subscription_count(), 0);
  EXPECT_FALSE(msg_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(msg_pub->is_activated());
  msg_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create the bridge node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Create the message
  dsr_msgs::msg::Node node_msg;
  node_msg.header.frame_id = "test";
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.attributes = {"level", "5", "1"};

  // Publish the message
  msg_pub->publish(node_msg);

  // Spin the dsr_bridge node
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationEdge) {
  rclcpp::init(0, nullptr);

  // Create a publisher node
  auto pub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("edge_publisher");
  pub_node->configure();

  // Create a publisher for the edge_msg
  auto msg_pub = pub_node->create_publisher<dsr_msgs::msg::Edge>("edges", 1);
  ASSERT_EQ(msg_pub->get_subscription_count(), 0);
  EXPECT_FALSE(msg_pub->is_activated());
  // Activate the publisher
  pub_node->activate();
  EXPECT_TRUE(msg_pub->is_activated());
  msg_pub->on_activate();
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node->get_node_base_interface());});

  // Create the bridge node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Create the message
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.header.frame_id = "test";
  edge_msg.parent = "robot";
  edge_msg.child = "person";
  edge_msg.type = "is";

  // Publish the message
  msg_pub->publish(edge_msg);

  // Spin the dsr_bridge node
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

int main(int argc, char ** argv)
{
  // QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
