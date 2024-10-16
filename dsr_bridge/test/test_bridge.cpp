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
};

TEST_F(DsrUtilTest, DSRBridgeFixtureConfigure) {
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
