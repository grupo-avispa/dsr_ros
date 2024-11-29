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

  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    return dsr_bridge::DSRBridge::add_node<node_type>(name);
  }

  bool delete_node(const std::string & name)
  {
    return dsr_bridge::DSRBridge::delete_node(name);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & parent, const std::string & child)
  {
    return dsr_bridge::DSRBridge::add_edge<edge_type>(parent, child);
  }

  bool delete_edge(std::string from, std::string to, std::string edge_type)
  {
    return dsr_bridge::DSRBridge::delete_edge(from, to, edge_type);
  }

  std::vector<dsr_msgs::msg::Edge> get_lost_edges()
  {
    return lost_edges_;
  }
};

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSCreateNode) {
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

  // Create a message with different source
  dsr_msgs::msg::Node node_msg;
  node_msg.header.frame_id = "test";
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.attributes = {"level", "5", "1"};

  // Publish and spin
  msg_pub->publish(node_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The node should be in the graph because the source is different
  EXPECT_TRUE(bridge_node->get_graph()->get_node("robot_name").has_value());

  // Now, create a message with the same source
  node_msg.header.frame_id = "robot";
  node_msg.name = "robot_name2";
  node_msg.type = "robot";

  // Publish and spin
  msg_pub->publish(node_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // The node should not be in the graph because the message comes from the same source
  EXPECT_FALSE(bridge_node->get_graph()->get_node("robot_name2").has_value());

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSModifyNode) {
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

  // Insert a node in the graph
  auto new_node = DSR::Node::create<robot_node_type>("robot_name");
  bridge_node->get_graph()->insert_node(new_node);
  bridge_node->get_graph()->add_or_modify_attrib_local<width_att>(new_node, 35);
  bridge_node->get_graph()->update_node(new_node);

  // Create a message with different source
  dsr_msgs::msg::Node node_msg;
  node_msg.header.frame_id = "test";
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.attributes = {"level", "5", "1"};

  // Publish and spin
  msg_pub->publish(node_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The node should be in the graph
  auto node = bridge_node->get_graph()->get_node("robot_name");
  EXPECT_TRUE(node.has_value());
  auto attributes = node.value().attrs();
  auto search = attributes.find("width");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 35);
  search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);

  // Now, create a message with the same source
  node_msg.header.frame_id = "robot";
  node_msg.name = "robot_name";
  node_msg.type = "robot";

  // Publish and spin
  msg_pub->publish(node_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSDeleteNode) {
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

  // Insert a node in the graph
  auto new_node = DSR::Node::create<robot_node_type>("robot_name");
  bridge_node->get_graph()->insert_node(new_node);

  // Create a message with different source
  dsr_msgs::msg::Node node_msg;
  node_msg.header.frame_id = "test";
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.deleted = true;
  node_msg.attributes = {"level", "5", "1"};

  // Publish and spin
  msg_pub->publish(node_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The node should not be in the graph
  EXPECT_FALSE(bridge_node->get_graph()->get_node("robot_name").has_value());

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSCreateEdge) {
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

  // Create a message with different source
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.header.frame_id = "test";
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";

  // Publish and spin
  msg_pub->publish(edge_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The edge should not be in the graph because the nodes are not in the graph
  EXPECT_FALSE(bridge_node->get_graph()->get_node("robot_parent").has_value());
  EXPECT_FALSE(bridge_node->get_graph()->get_node("robot_child").has_value());
  EXPECT_EQ(bridge_node->get_lost_edges().size(), 1);
  EXPECT_EQ(bridge_node->get_lost_edges()[0].parent, "robot_parent");
  EXPECT_EQ(bridge_node->get_lost_edges()[0].child, "robot_child");

  // Now, insert the nodes in the graph
  auto new_node = DSR::Node::create<robot_node_type>("robot_parent");
  bridge_node->get_graph()->insert_node(new_node);
  new_node = DSR::Node::create<person_node_type>("robot_child");
  bridge_node->get_graph()->insert_node(new_node);

  // Create a message with different source
  edge_msg.header.frame_id = "test";
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";

  // Publish and spin
  msg_pub->publish(edge_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // The edge should be in the graph because the source is different
  EXPECT_TRUE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent").value().id(),
      bridge_node->get_graph()->get_node("robot_child").value().id(), "is").has_value());

  // Now, insert new nodes in the graph
  new_node = DSR::Node::create<robot_node_type>("robot_parent2");
  bridge_node->get_graph()->insert_node(new_node);
  new_node = DSR::Node::create<person_node_type>("robot_child2");
  bridge_node->get_graph()->insert_node(new_node);

  // Now, create a message with the same source
  edge_msg.header.frame_id = "robot";
  edge_msg.parent = "robot_parent2";
  edge_msg.child = "robot_child2";
  edge_msg.type = "has";

  // Publish and spin
  msg_pub->publish(edge_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // The edge should not be in the graph because the message comes from the same source
  EXPECT_FALSE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent2").value().id(),
      bridge_node->get_graph()->get_node("robot_child2").value().id(), "has").has_value());

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSModifyEdge) {
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

  // Insert the nodes and the edge in the graph
  auto new_node = DSR::Node::create<robot_node_type>("robot_parent");
  bridge_node->get_graph()->insert_node(new_node);
  new_node = DSR::Node::create<person_node_type>("robot_child");
  bridge_node->get_graph()->insert_node(new_node);
  auto new_edge = DSR::Edge::create<is_edge_type>(
    bridge_node->get_graph()->get_node("robot_parent").value().id(),
    bridge_node->get_graph()->get_node("robot_child").value().id());
  bridge_node->get_graph()->insert_or_assign_edge(new_edge);
  bridge_node->get_graph()->add_or_modify_attrib_local<width_att>(new_edge, 35);
  bridge_node->get_graph()->insert_or_assign_edge(new_edge);

  // Check that the edge is in the graph
  EXPECT_TRUE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent").value().id(),
      bridge_node->get_graph()->get_node("robot_child").value().id(), "is").has_value());

  // Create the message
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.header.frame_id = "test";
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";
  edge_msg.attributes = {"level", "5", "1"};

  // Publish and spin
  msg_pub->publish(edge_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The edge should be in the graph
  auto edge = bridge_node->get_graph()->get_edge(
    bridge_node->get_graph()->get_node("robot_parent").value().id(),
    bridge_node->get_graph()->get_node("robot_child").value().id(), "is");
  EXPECT_TRUE(edge.has_value());
  auto attributes = edge.value().attrs();
  auto search = attributes.find("width");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 35);
  search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationFromROSDeleteEdge) {
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

  // Insert the nodes and the edge in the graph
  auto new_node = DSR::Node::create<robot_node_type>("robot_parent");
  bridge_node->get_graph()->insert_node(new_node);
  new_node = DSR::Node::create<person_node_type>("robot_child");
  bridge_node->get_graph()->insert_node(new_node);
  auto new_edge = DSR::Edge::create<is_edge_type>(
    bridge_node->get_graph()->get_node("robot_parent").value().id(),
    bridge_node->get_graph()->get_node("robot_child").value().id());
  bridge_node->get_graph()->insert_or_assign_edge(new_edge);

  // Check that the edge is in the graph
  EXPECT_TRUE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent").value().id(),
      bridge_node->get_graph()->get_node("robot_child").value().id(), "is").has_value());

  // Create the message
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.header.frame_id = "test";
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";
  edge_msg.deleted = true;

  // Publish and spin
  msg_pub->publish(edge_msg);
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message publisher should have a subscription
  EXPECT_EQ(msg_pub->get_subscription_count(), 1);

  // The edge should not be in the graph
  EXPECT_FALSE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent").value().id(),
      bridge_node->get_graph()->get_node("robot_child").value().id(), "is").has_value());

  // Deactivate the nodes
  bridge_node->deactivate();
  pub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  pub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationToROSNode) {
  rclcpp::init(0, nullptr);

  // Create a subscriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_subscriber");
  sub_node->configure();
  sub_node->activate();

  // Create a subscriber for the node_msgs
  // The first count is for created, the second for modified and the third for deleted
  bool msg_received = false;
  int count = 0;
  auto msg_sub = sub_node->create_subscription<dsr_msgs::msg::Node>(
    "nodes", 1,
    [&](const dsr_msgs::msg::Node msg) {
      msg_received = true;
      EXPECT_EQ(msg.name, "robot_name");
      EXPECT_EQ(msg.type, "robot");
      if (count == 1) {
        EXPECT_EQ(msg.attributes[0], "level");
        EXPECT_EQ(msg.attributes[1], "42");
        EXPECT_EQ(msg.attributes[2], "1");
      }
      if (count == 2) {
        EXPECT_TRUE(msg.deleted);
      }
      RCLCPP_INFO(sub_node->get_logger(), "Message received");
      count++;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Create the bridge node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Add a DRS node
  auto dsr_parent_node = bridge_node->add_node<robot_node_type>("robot_name");
  EXPECT_TRUE(bridge_node->get_graph()->get_node("robot_name").has_value());

  // Spin
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message subscriber should have a publisher
  EXPECT_EQ(msg_sub->get_publisher_count(), 1);

  // Modify the node
  auto node = bridge_node->get_graph()->get_node("robot_name");
  bridge_node->get_graph()->add_or_modify_attrib_local<level_att>(node.value(), 42);
  bridge_node->get_graph()->update_node(node.value());

  // Spin
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Delete the node
  bridge_node->delete_node("robot_name");

  // Deactivate the nodes
  bridge_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  sub_thread.join();
}

TEST_F(DsrUtilTest, DSRBridgeIntegrationToROSEdge) {
  rclcpp::init(0, nullptr);

  // Create a subscriber node
  auto sub_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node_subscriber");
  sub_node->configure();
  sub_node->activate();

  // Create a subscriber for the edge_msgs
  // The first count is for created, the second for modified and the third for deleted
  bool msg_received = false;
  int count = 0;
  auto msg_sub = sub_node->create_subscription<dsr_msgs::msg::Edge>(
    "edges", 1,
    [&](const dsr_msgs::msg::Edge msg) {
      msg_received = true;
      EXPECT_EQ(msg.parent, "robot_parent");
      EXPECT_EQ(msg.child, "robot_child");
      EXPECT_EQ(msg.type, "is");
      if (count == 1) {
        EXPECT_EQ(msg.attributes[0], "level");
        EXPECT_EQ(msg.attributes[1], "42");
        EXPECT_EQ(msg.attributes[2], "1");
      }
      if (count == 2) {
        EXPECT_TRUE(msg.deleted);
      }
      RCLCPP_INFO(sub_node->get_logger(), "Message received");
      count++;
    });
  auto sub_thread = std::thread([&]() {rclcpp::spin(sub_node->get_node_base_interface());});

  // Create the bridge node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Add the DRS nodes and edge
  auto dsr_parent_node = bridge_node->add_node<robot_node_type>("robot_parent");
  auto dsr_child_node = bridge_node->add_node<robot_node_type>("robot_child");
  auto dsr_edge = bridge_node->add_edge<is_edge_type>("robot_parent", "robot_child");
  EXPECT_TRUE(
    bridge_node->get_graph()->get_edge(
      bridge_node->get_graph()->get_node("robot_parent").value().id(),
      bridge_node->get_graph()->get_node("robot_child").value().id(), "is").has_value());

  // Spin
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Check the results: now, the message subscriber should have a publisher
  EXPECT_EQ(msg_sub->get_publisher_count(), 1);

  // Modify the edge
  auto parent_node = bridge_node->get_graph()->get_node("robot_parent");
  auto child_node = bridge_node->get_graph()->get_node("robot_child");
  auto edge = bridge_node->get_graph()->get_edge(
    parent_node.value().id(), child_node.value().id(), "is");
  bridge_node->get_graph()->add_or_modify_attrib_local<level_att>(edge.value(), 42);
  bridge_node->get_graph()->insert_or_assign_edge(edge.value());

  // Spin
  rclcpp::spin_some(bridge_node->get_node_base_interface());

  // Delete the edge
  bridge_node->delete_edge("robot_parent", "robot_child", "is");

  // Deactivate the nodes
  bridge_node->deactivate();
  sub_node->deactivate();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  sub_thread.join();
}

int main(int argc, char ** argv)
{
  // QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
