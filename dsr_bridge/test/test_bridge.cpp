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
#include "dsr_bridge/dsr_bridge.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "lifecycle_msgs/msg/state.hpp"

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

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & parent, const std::string & child)
  {
    return dsr_bridge::DSRBridge::add_edge<edge_type>(parent, child);
  }

  DSR::Node from_msg(const dsr_msgs::msg::Node & msg)
  {
    return dsr_bridge::DSRBridge::from_msg(msg);
  }

  dsr_msgs::msg::Node to_msg(const DSR::Node & node, bool deleted)
  {
    return dsr_bridge::DSRBridge::to_msg(node, deleted);
  }

  DSR::Edge from_msg(const dsr_msgs::msg::Edge & msg)
  {
    return dsr_bridge::DSRBridge::from_msg(msg);
  }

  dsr_msgs::msg::Edge to_msg(const DSR::Edge & edge, bool deleted)
  {
    return dsr_bridge::DSRBridge::to_msg(edge, deleted);
  }

  std::vector<dsr_msgs::msg::Edge> get_lost_edges()
  {
    return lost_edges_;
  }

  void store_lost_edge(const dsr_msgs::msg::Edge & edge)
  {
    lost_edges_.push_back(edge);
  }

  void insert_lost_edges()
  {
    dsr_bridge::DSRBridge::insert_lost_edges();
  }
};

TEST_F(DsrUtilTest, DSRBridgeConfigure) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  const auto state_after_configure = bridge_node->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  bridge_node->activate();
  bridge_node->deactivate();
  bridge_node->cleanup();
  bridge_node->shutdown();
}

TEST_F(DsrUtilTest, DSRBridgeCreateDSRNode) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  // Create the message
  dsr_msgs::msg::Node node_msg;
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.attributes = {"level", "5", "1"};

  // Create the DSR node
  auto dsr_node = bridge_node->from_msg(node_msg);
  EXPECT_EQ(dsr_node.name(), node_msg.name);
  EXPECT_EQ(dsr_node.type(), node_msg.type);
  auto attributes = dsr_node.attrs();
  auto search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);

  // Update the message with a wrong type
  node_msg.type = "wrong";
  EXPECT_THROW(bridge_node->from_msg(node_msg), std::runtime_error);
}

TEST_F(DsrUtilTest, DSRBridgeCreateMsgNode) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Create the DSR node
  auto dsr_node = DSR::Node::create<robot_node_type>("robot_name");
  bridge_node->get_graph()->add_or_modify_attrib_local<level_att>(dsr_node, 5);

  // Create the message
  auto node_msg = bridge_node->to_msg(dsr_node, true);
  EXPECT_EQ(node_msg.name, dsr_node.name());
  EXPECT_EQ(node_msg.type, dsr_node.type());
  auto attributes = dsr_node.attrs();
  auto search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);
  EXPECT_TRUE(node_msg.deleted);
}

TEST_F(DsrUtilTest, DSRBridgeCreateDSREdge) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Add the DRS nodes
  auto dsr_parent_node = bridge_node->add_node<robot_node_type>("robot_parent");
  auto dsr_child_node = bridge_node->add_node<robot_node_type>("robot_child");

  // Create the message
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";
  edge_msg.attributes = {"source", "robot", "0"};

  // Create the DSR edge
  auto dsr_edge = bridge_node->from_msg(edge_msg);
  EXPECT_EQ(dsr_edge.from(), bridge_node->get_graph()->get_node("robot_parent").value().id());
  EXPECT_EQ(dsr_edge.to(), bridge_node->get_graph()->get_node("robot_child").value().id());
  EXPECT_EQ(dsr_edge.type(), "is");
  auto attributes = dsr_edge.attrs();
  auto search = attributes.find("source");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<std::string>(search->second.value()), "robot");

  // Update the message with a wrong type
  edge_msg.type = "wrong";
  EXPECT_THROW(bridge_node->from_msg(edge_msg), std::runtime_error);
}

TEST_F(DsrUtilTest, DSRBridgeCreateMsgEdge) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  // Add the DRS nodes and edge
  auto dsr_parent_node = bridge_node->add_node<robot_node_type>("robot_parent");
  auto dsr_child_node = bridge_node->add_node<robot_node_type>("robot_child");
  auto dsr_edge = bridge_node->add_edge<is_edge_type>("robot_parent", "robot_child");

  // Create the message
  auto edge_msg = bridge_node->to_msg(dsr_edge.value(), true);
  EXPECT_EQ(edge_msg.parent, "robot_parent");
  EXPECT_EQ(edge_msg.child, "robot_child");
  EXPECT_EQ(edge_msg.type, "is");
  auto attributes = dsr_edge.value().attrs();
  auto search = attributes.find("source");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<std::string>(search->second.value()), "robot");
  EXPECT_TRUE(edge_msg.deleted);
}

TEST_F(DsrUtilTest, DSRBridgeInsertLostEdges) {
  // Create the node
  auto bridge_node = std::make_shared<DSRBridgeFixture>();
  bridge_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  bridge_node->configure();
  bridge_node->activate();

  EXPECT_TRUE(bridge_node->get_lost_edges().empty());

  // Create the message and store it
  dsr_msgs::msg::Edge edge_msg;
  edge_msg.parent = "robot_parent";
  edge_msg.child = "robot_child";
  edge_msg.type = "is";
  edge_msg.attributes = {"source", "robot", "0"};
  bridge_node->store_lost_edge(edge_msg);
  EXPECT_EQ(bridge_node->get_lost_edges().size(), 1);

  // Add the DRS nodes
  auto dsr_parent_node = bridge_node->add_node<robot_node_type>("robot_parent");
  auto dsr_child_node = bridge_node->add_node<robot_node_type>("robot_child");

  // Insert the lost edges
  bridge_node->insert_lost_edges();

  // Check the results
  EXPECT_EQ(bridge_node->get_lost_edges().size(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
