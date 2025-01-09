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
#include "dsr_util/node_agent.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "utils/test_dsr_setup.hpp"

class AgentNodeFixture : public dsr_util::NodeAgent
{
public:
  explicit AgentNodeFixture(std::string name)
  : dsr_util::NodeAgent(name)
  {
  }
  ~AgentNodeFixture() = default;

  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    return dsr_util::NodeAgent::add_node<node_type>(name);
  }

  template<typename node_type, typename edge_type>
  std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
  add_node_with_edge(
    const std::string & name, const std::string & connecting_node_name, const bool reversed = false)
  {
    return dsr_util::NodeAgent::add_node_with_edge<node_type, edge_type>(
      name, connecting_node_name, reversed);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & from, const std::string & to)
  {
    return dsr_util::NodeAgent::add_edge<edge_type>(from, to);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(uint64_t from, uint64_t to)
  {
    return dsr_util::NodeAgent::add_edge<edge_type>(from, to);
  }

  bool delete_node(uint64_t id)
  {
    return dsr_util::NodeAgent::delete_node(id);
  }

  bool delete_node(const std::string & name)
  {
    return dsr_util::NodeAgent::delete_node(name);
  }

  bool delete_edge(uint64_t from, uint64_t to, std::string edge_type)
  {
    return dsr_util::NodeAgent::delete_edge(from, to, edge_type);
  }

  bool delete_edge(const std::string & from, const std::string & to, std::string edge_type)
  {
    return dsr_util::NodeAgent::delete_edge(from, to, edge_type);
  }

  template<typename edge_type>
  bool replace_edge(uint64_t from, uint64_t to, std::string old_edge)
  {
    return dsr_util::NodeAgent::replace_edge<edge_type>(from, to, old_edge);
  }

  template<typename edge_type>
  bool replace_edge(const std::string & from, const std::string & to, std::string old_edge)
  {
    return dsr_util::NodeAgent::replace_edge<edge_type>(from, to, old_edge);
  }

  void update_rt_attributes(
    DSR::Node & from, DSR::Node & to,
    const geometry_msgs::msg::Transform & msg)
  {
    dsr_util::NodeAgent::update_rt_attributes(from, to, msg);
  }

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }
};

TEST_F(DsrUtilTest, agentNodeConfigure) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_agent->activate();
  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeConfigureEmpty) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNode) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add a node
  auto node = node_agent->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(node_agent->get_graph()->get_node("test_node").has_value());

  // Try to add the same node again
  auto node2 = node_agent->add_node<robot_node_type>("test_node");

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNodeWithEdge) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add a node with an edge
  auto [node, edge] = node_agent->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", false);
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(node_agent->get_graph()->get_node("test_node").has_value());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().from()).value().name(), "world");
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().to()).value().name(), "test_node");
  EXPECT_EQ(edge.value().type(), "is");

  // Try to add the same node again
  auto [node2, edge2] = node_agent->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", false);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddNodeWithEdgeReversed) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add a node with an edge
  auto [node, edge] = node_agent->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", true);
  EXPECT_TRUE(node.has_value());
  EXPECT_EQ(node.value().name(), "test_node");
  EXPECT_EQ(node.value().type(), "robot");
  EXPECT_TRUE(node_agent->get_graph()->get_node("test_node").has_value());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().from()).value().name(), "test_node");
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().to()).value().name(), "world");
  EXPECT_EQ(edge.value().type(), "is");

  // Try to add the same node again
  auto [node2, edge2] = node_agent->add_node_with_edge<robot_node_type, is_edge_type>(
    "test_node", "world", true);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddEdgeId) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().from()).value().name(), "parent_node");
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().to()).value().name(), "child_node");
  EXPECT_EQ(edge.value().type(), "is");

  // Add an edge with a parent node that doesn't exist
  auto edge2 = node_agent->add_edge<is_edge_type>(2045, child_node.value().id());

  // Add an edge with a child node that doesn't exist
  auto edge3 = node_agent->add_edge<is_edge_type>(parent_node.value().id(), 505250);

  // Try to add the same edge again
  auto edge4 =
    node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeAddEdgeStr) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>("parent_node", "child_node");
  EXPECT_TRUE(edge.has_value());
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().from()).value().name(), "parent_node");
  EXPECT_EQ(node_agent->get_graph()->get_node(edge.value().to()).value().name(), "child_node");
  EXPECT_EQ(edge.value().type(), "is");

  // Add an edge with a parent node that doesn't exist
  auto edge2 = node_agent->add_edge<is_edge_type>("parent_fake_node", "child_node");

  // Add an edge with a child node that doesn't exist
  auto edge3 = node_agent->add_edge<is_edge_type>("parent_node", "child_fake_node");

  // Try to add the same edge again
  auto edge4 = node_agent->add_edge<is_edge_type>("parent_node", "child_node");

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteNodeId) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add a node
  auto node = node_agent->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());

  // Delete the node
  EXPECT_TRUE(node_agent->delete_node(node.value().id()));

  // Delete a node that doesn't exist
  EXPECT_FALSE(node_agent->delete_node(2045));

  // Try to delete the node again
  EXPECT_FALSE(node_agent->delete_node(node.value().id()));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteNodeStr) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add a node
  auto node = node_agent->add_node<robot_node_type>("test_node");
  EXPECT_TRUE(node.has_value());

  // Delete the node
  EXPECT_TRUE(node_agent->delete_node(node.value().name()));

  // Delete a node that doesn't exist
  EXPECT_FALSE(node_agent->delete_node("fake_node"));

  // Delete the node again
  EXPECT_FALSE(node_agent->delete_node(node.value().name()));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteEdgeId) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Delete the edge
  EXPECT_TRUE(node_agent->delete_edge(parent_node.value().id(), child_node.value().id(), "is"));

  // Delete an edge with a parent node that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge(2045, child_node.value().id(), "is"));

  // Delete an edge with a child node that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge(parent_node.value().id(), 505250, "is"));

  // Delete an edge with an edge that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge(parent_node.value().id(), child_node.value().id(), "is"));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeDeleteEdgeStr) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Delete the edge
  EXPECT_TRUE(node_agent->delete_edge("parent_node", "child_node", "is"));

  // Delete an edge with a parent node that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge("parent_fake_node", "child_node", "is"));

  // Delete an edge with a child node that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge("parent_node", "child_fake_node", "is"));

  // Delete an edge with an edge that doesn't exist
  EXPECT_FALSE(node_agent->delete_edge("parent_node", "child_node", "is"));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeReplaceEdgeId) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Replace the edge
  EXPECT_TRUE(
    node_agent->replace_edge<has_edge_type>(
      parent_node.value().id(), child_node.value().id(), "is"));

  // Replace an edge with a parent node that doesn't exist
  EXPECT_FALSE(
    node_agent->replace_edge<has_edge_type>(2045, child_node.value().id(), "is"));

  // Replace an edge with a child node that doesn't exist
  EXPECT_FALSE(
    node_agent->replace_edge<has_edge_type>(parent_node.value().id(), 505250, "is"));

  // Replace an edge with an edge that doesn't exist
  EXPECT_FALSE(
    node_agent->replace_edge<has_edge_type>(
      parent_node.value().id(), child_node.value().id(), "is"));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeReplaceEdgeStr) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes and an edge
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
  auto edge = node_agent->add_edge<is_edge_type>(parent_node.value().id(), child_node.value().id());

  // Replace the edge
  EXPECT_TRUE(node_agent->replace_edge<has_edge_type>("parent_node", "child_node", "is"));

  // Replace an edge with a parent node that doesn't exist
  EXPECT_FALSE(node_agent->replace_edge<has_edge_type>("parent_fake_node", "child_node", "is"));

  // Replace an edge with a child node that doesn't exist
  EXPECT_FALSE(node_agent->replace_edge<has_edge_type>("parent_node", "child_fake_node", "is"));

  // Replace an edge with an edge that doesn't exist
  EXPECT_FALSE(node_agent->replace_edge<has_edge_type>("parent_node", "child_node", "is"));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeUpdateRTAttributes) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Add two nodes
  auto parent_node = node_agent->add_node<robot_node_type>("parent_node");
  EXPECT_TRUE(parent_node.has_value());
  auto child_node = node_agent->add_node<robot_node_type>("child_node");
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
  node_agent->update_rt_attributes(parent_node.value(), child_node.value(), msg);

  // Check the RT attributes
  auto rt_edge = node_agent->get_graph()->get_edge(
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

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, agentNodeSaveDSR) {
  // Create the node
  auto node_agent = std::make_shared<AgentNodeFixture>("test_node");
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->configure();
  node_agent->activate();

  // Create the client service
  auto req = std::make_shared<dsr_msgs::srv::SaveDSR::Request>();
  auto pkg = ament_index_cpp::get_package_share_directory("dsr_util");
  req->dsr_url = pkg + "/test/test_save_dsr.json";
  auto client = node_agent->create_client<dsr_msgs::srv::SaveDSR>("save_dsr");

  // Wait for the service to be available
  ASSERT_TRUE(client->wait_for_service());

  // Call the service
  auto result = client->async_send_request(req);

  // Wait for the result
  auto resp = std::make_shared<dsr_msgs::srv::SaveDSR::Response>();
  if (rclcpp::spin_until_future_complete(node_agent, result) == rclcpp::FutureReturnCode::SUCCESS) {
    std::cout << "Service call succeeded" << std::endl;
    resp = result.get();
  } else {
    std::cout << "Service call failed" << std::endl;
  }

  // Check the result
  EXPECT_TRUE(resp->result);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
