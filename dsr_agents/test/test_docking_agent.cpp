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
#include "dsr_agents/docking_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_util/simple_action_server.hpp"


class DockingAgentFixture : public dsr_agents::DockingAgent
{
public:
  DockingAgentFixture()
  : dsr_agents::DockingAgent()
  {
  }

  ~DockingAgentFixture() = default;

  bool get_goal_from_dsr(DSR::Node action_node)
  {
    return DockingAgent::get_goal_from_dsr(action_node);
  }

  nav2_msgs::action::DockRobot::Goal get_goal()
  {
    return goal_;
  }

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }
};

class DummyDockingServer : rclcpp::Node
{
public:
  using ActionT = nav2_msgs::action::DockRobot;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  DummyDockingServer()
  : Node("dummy_docking")
  {
    action_server_ = std::make_shared<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "dock_robot", std::bind(&DummyDockingServer::executeCallback, this),
      nullptr, std::chrono::milliseconds(500), true);

    action_server_->activate();
  }

  ~DummyDockingServer() = default;

  void setReturn(const bool rtn)
  {
    return_state_ = rtn;
  }

  void setToggle()
  {
    toggle_ = true;
  }

  void executeCallback()
  {
    auto result = std::make_shared<typename ActionT::Result>();
    auto goal = action_server_->get_current_goal();
    (void)goal;

    bool rtn = return_state_;
    if (toggle_) {
      return_state_ = !return_state_;
    }

    if (rtn) {
      action_server_->succeeded_current(result);
      return;
    }
    action_server_->terminate_current(result);
  }

protected:
  std::shared_ptr<ActionServer> action_server_;
  bool return_state_{true};
  bool toggle_{false};
};

TEST_F(DsrUtilTest, dockingAgentGetGoalFromDSR) {
  auto node_agent = std::make_shared<DockingAgentFixture>();
  auto dummy_docking_node = std::make_shared<DummyDockingServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("dock_robot"));

  node_agent->configure();
  node_agent->activate();

  // Insert a node with the goal in the graph
  auto new_node = DSR::Node::create<dock_node_type>("dock");
  node_agent->get_graph()->insert_node(new_node);
  node_agent->get_graph()->add_or_modify_attrib_local<dock_id_att>(
    new_node, static_cast<std::string>("test_id"));
  node_agent->get_graph()->update_node(new_node);

  // Check the results
  EXPECT_TRUE(node_agent->get_goal_from_dsr(new_node));
  EXPECT_EQ(node_agent->get_goal().dock_id, "test_id");

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, dockingAgentGetGoalFromDSRMisssing) {
  auto node_agent = std::make_shared<DockingAgentFixture>();
  auto dummy_docking_node = std::make_shared<DummyDockingServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("dock_robot"));

  node_agent->configure();
  node_agent->activate();

  // Insert a node with the goal in the graph but without values
  auto new_node = DSR::Node::create<dock_node_type>("dock");
  node_agent->get_graph()->insert_node(new_node);

  // Check the results
  EXPECT_FALSE(node_agent->get_goal_from_dsr(new_node));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
