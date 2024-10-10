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
#include "dsr_util/agent_node.hpp"
#include "dsr_util/qt_executor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "test_dsr_setup.hpp"

class AgentNodeFixture : public dsr_util::AgentNode
{
public:
  explicit AgentNodeFixture(std::string name)
  : dsr_util::AgentNode(name)
  {
  }
  ~AgentNodeFixture() = default;

  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    return dsr_util::AgentNode::add_node<node_type>(name);
  }

  template<typename node_type, typename edge_type>
  std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
  add_node_with_edge(
    const std::string & name, const std::string & connecting_node_name, const bool reversed = false)
  {
    return dsr_util::AgentNode::add_node_with_edge<node_type, edge_type>(
      name, connecting_node_name, reversed);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & from, const std::string & to)
  {
    return dsr_util::AgentNode::add_edge<edge_type>(from, to);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(uint64_t from, uint64_t to)
  {
    return dsr_util::AgentNode::add_edge<edge_type>(from, to);
  }

  bool delete_node(uint64_t id)
  {
    return dsr_util::AgentNode::delete_node(id);
  }

  bool delete_node(const std::string & name)
  {
    return dsr_util::AgentNode::delete_node(name);
  }

  bool delete_edge(uint64_t from, uint64_t to, std::string edge_type)
  {
    return dsr_util::AgentNode::delete_edge(from, to, edge_type);
  }

  bool delete_edge(const std::string & from, const std::string & to, std::string edge_type)
  {
    return dsr_util::AgentNode::delete_edge(from, to, edge_type);
  }

  template<typename edge_type>
  bool replace_edge(uint64_t from, uint64_t to, std::string old_edge)
  {
    return dsr_util::AgentNode::replace_edge<edge_type>(from, to, old_edge);
  }

  template<typename edge_type>
  bool replace_edge(const std::string & from, const std::string & to, std::string old_edge)
  {
    return dsr_util::AgentNode::replace_edge<edge_type>(from, to, old_edge);
  }

  void update_rt_attributes(
    DSR::Node & from, DSR::Node & to,
    const geometry_msgs::msg::Transform & msg)
  {
    dsr_util::AgentNode::update_rt_attributes(from, to, msg);
  }

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }
};

TEST_F(DsrUtilTest, agentNodeConfigure)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();
  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeConfigureEmpty)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->configure();
  agent_node->activate();
  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNode)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add a node
  auto node = agent_node->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(agent_node->get_graph()->get_node("test_node").has_value());

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNodeWithEdge)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add a node with an edge
  auto [node, edge] = agent_node->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", false);
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(agent_node->get_graph()->get_node("test_node").has_value());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().from()).value().name(), "world");
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().to()).value().name(), "test_node");
  EXPECT_EQ(edge.value().type(), "is");

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNodeWithEdgeReversed)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add a node with an edge
  auto [node, edge] = agent_node->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", true);
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(agent_node->get_graph()->get_node("test_node").has_value());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().from()).value().name(), "test_node");
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().to()).value().name(), "world");
  EXPECT_EQ(edge.value().type(), "is");

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddEdgeStr)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>("parent_node", "child_node");
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().from()).value().name(), "parent_node");
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().to()).value().name(), "child_node");
  EXPECT_EQ(edge.value().type(), "is");

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddEdgeId)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().from()).value().name(), "parent_node");
  EXPECT_EQ(agent_node->get_graph()->get_node(edge.value().to()).value().name(), "child_node");
  EXPECT_EQ(edge.value().type(), "is");

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteNodeId)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add a node
  auto node = agent_node->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());

  // Delete the node
  EXPECT_TRUE(agent_node->delete_node(node.value().id()));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteNodeStr)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add a node
  auto node = agent_node->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());

  // Delete the node
  EXPECT_TRUE(agent_node->delete_node(node.value().name()));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteEdgeId)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Delete the edge
  EXPECT_TRUE(agent_node->delete_edge(parent_node.value().id(), child_node.value().id(), "is"));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteEdgeStr)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Delete the edge
  EXPECT_TRUE(agent_node->delete_edge("parent_node", "child_node", "is"));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeReplaceEdgeId)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Replace the edge
  EXPECT_TRUE(
    agent_node->replace_edge<has_edge_type>(
      parent_node.value().id(), child_node.value().id(), "is"));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeReplaceEdgeStr)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes and an edge
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  auto edge = agent_node->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Replace the edge
  EXPECT_TRUE(agent_node->replace_edge<has_edge_type>("parent_node", "child_node", "is"));

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, agentNodeUpdateRTAttributes)
{
  // Create the node
  auto agent_node = std::make_shared<AgentNodeFixture>("test_node");
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Add two nodes
  auto parent_node = agent_node->add_node<robot_node_type>("parent_node");
  EXPECT_TRUE(parent_node.has_value());
  auto child_node = agent_node->add_node<robot_node_type>("child_node");
  EXPECT_TRUE(child_node.has_value());

  // Update the RT attributes
  geometry_msgs::msg::Transform msg;
  msg.translation.x = 1.0;
  msg.translation.y = 2.0;
  msg.translation.z = 3.0;
  msg.rotation.x = 0.0;
  msg.rotation.y = 0.0;
  msg.rotation.z = 0.0;
  msg.rotation.w = 1.0;
  agent_node->update_rt_attributes(parent_node.value(), child_node.value(), msg);

  // Check the RT attributes
  auto rt_edge = agent_node->get_graph()->get_edge(
    parent_node.value().id(), child_node.value().id(), "RT");
  EXPECT_TRUE(rt_edge.has_value());

  auto attributes = rt_edge.value().attrs();
  auto trans = std::get<std::vector<float>>(attributes["rt_translation"].value());
  EXPECT_FLOAT_EQ(trans[0], 1.0);
  EXPECT_FLOAT_EQ(trans[1], 2.0);
  EXPECT_FLOAT_EQ(trans[2], 3.0);

  auto rot = std::get<std::vector<float>>(attributes["rt_rotation_euler_xyz"].value());
  EXPECT_FLOAT_EQ(rot[0], 0.0);
  EXPECT_FLOAT_EQ(rot[1], 0.0);
  EXPECT_FLOAT_EQ(rot[2], 0.0);

  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}


int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
