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

  DSR::Node from_msg(const dsr_msgs::msg::Node & msg)
  {
    return dsr_bridge::DSRBridge::from_msg(msg);
  }

  dsr_msgs::msg::Node to_msg(const DSR::Node & node, bool deleted)
  {
    return dsr_bridge::DSRBridge::to_msg(node, deleted);
  }
};

TEST_F(DsrUtilTest, DSRBridgeConfigure) {
  // Create the node
  auto agent_node = std::make_shared<DSRBridgeFixture>();
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  const auto state_after_configure = agent_node->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  agent_node->activate();
  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, DSRBridgeCreateDSRNode) {
  // Create the node
  auto agent_node = std::make_shared<DSRBridgeFixture>();
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  // Create the message
  dsr_msgs::msg::Node node_msg;
  node_msg.name = "robot_name";
  node_msg.type = "robot";
  node_msg.attributes = {"level", "5", "1"};

  // Create the DSR node
  auto dsr_node = agent_node->from_msg(node_msg);
  EXPECT_EQ(dsr_node.name(), node_msg.name);
  EXPECT_EQ(dsr_node.type(), node_msg.type);
  auto attributes = dsr_node.attrs();
  auto search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);
}

TEST_F(DsrUtilTest, DSRBridgeCreateMsgNode) {
  // Create the node
  auto agent_node = std::make_shared<DSRBridgeFixture>();
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->configure();
  agent_node->activate();

  // Create the DSR node
  auto dsr_node = DSR::Node::create<robot_node_type>("robot_name");
  agent_node->get_graph()->add_or_modify_attrib_local<level_att>(dsr_node, 5);

  // Create the message
  auto node_msg = agent_node->to_msg(dsr_node, true);
  EXPECT_EQ(node_msg.name, dsr_node.name());
  EXPECT_EQ(node_msg.type, dsr_node.type());
  auto attributes = dsr_node.attrs();
  auto search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 5);
  EXPECT_TRUE(node_msg.deleted);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
