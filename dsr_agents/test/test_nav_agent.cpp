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
#include "dsr_agents/nav_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"


class NavAgentFixture : public dsr_agents::NavigationAgent
{
public:
  NavAgentFixture()
  : dsr_agents::NavigationAgent()
  {
  }

  ~NavAgentFixture() = default;

  bool get_goal_from_dsr(DSR::Node action_node)
  {
    return NavigationAgent::get_goal_from_dsr(action_node);
  }

  nav2_msgs::action::NavigateToPose::Goal get_goal()
  {
    return goal_;
  }

  void on_feedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    return NavigationAgent::on_feedback(feedback);
  }

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }
};

class DummyNavigationServer : rclcpp::Node
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  DummyNavigationServer()
  : Node("dummy_navigator")
  {
    action_server_ = std::make_shared<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "navigate_to_pose", std::bind(&DummyNavigationServer::executeCallback, this),
      nullptr, std::chrono::milliseconds(500), true);

    action_server_->activate();
  }

  ~DummyNavigationServer() = default;

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

TEST_F(DsrUtilTest, navAgentGetGoalFromDSR) {
  auto node_agent = std::make_shared<NavAgentFixture>();
  auto dummy_navigator_node = std::make_shared<DummyNavigationServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("navigate_to_pose"));

  node_agent->configure();
  node_agent->activate();

  // Insert a node with the goal in the graph
  auto new_node = DSR::Node::create<move_node_type>("move");
  node_agent->get_graph()->insert_node(new_node);
  node_agent->get_graph()->add_or_modify_attrib_local<goal_x_att>(
    new_node, static_cast<float>(1.0));
  node_agent->get_graph()->add_or_modify_attrib_local<goal_y_att>(
    new_node, static_cast<float>(2.0));
  node_agent->get_graph()->add_or_modify_attrib_local<goal_angle_att>(
    new_node, static_cast<float>(3.0));
  node_agent->get_graph()->update_node(new_node);

  // Check the results
  EXPECT_TRUE(node_agent->get_goal_from_dsr(new_node));
  EXPECT_EQ(node_agent->get_goal().pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(node_agent->get_goal().pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(node_agent->get_goal().pose.pose.position.y, 2.0);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, navAgentGetGoalFromDSRMisssing) {
  auto node_agent = std::make_shared<NavAgentFixture>();
  auto dummy_navigator_node = std::make_shared<DummyNavigationServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("navigate_to_pose"));

  node_agent->configure();
  node_agent->activate();

  // Insert a node with the goal in the graph but without values
  auto new_node = DSR::Node::create<move_node_type>("move");
  node_agent->get_graph()->insert_node(new_node);

  // Check the results
  EXPECT_FALSE(node_agent->get_goal_from_dsr(new_node));

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
}

TEST_F(DsrUtilTest, navAgentOnFeedback) {
  auto node_agent = std::make_shared<NavAgentFixture>();
  auto dummy_navigator_node = std::make_shared<DummyNavigationServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("navigate_to_pose"));

  node_agent->configure();
  node_agent->activate();

  // Insert a robot node
  auto robot_node = DSR::Node::create<robot_node_type>("robot");
  node_agent->get_graph()->insert_node(robot_node);

  // Create a feedback message
  auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
  feedback->current_pose.pose.position.x = 1.0;
  feedback->current_pose.pose.position.y = 2.0;
  node_agent->on_feedback(feedback);

  // Check the results
  robot_node = node_agent->get_graph()->get_node("robot").value();
  auto attrs = robot_node.attrs();
  EXPECT_FLOAT_EQ(std::get<float>(attrs["pose_x"].value()), 1.0);
  EXPECT_FLOAT_EQ(std::get<float>(attrs["pose_y"].value()), 2.0);

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
