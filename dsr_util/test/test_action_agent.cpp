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
#include "dsr_util/action_agent.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "test_msgs/action/fibonacci.hpp"
#include "utils/test_dsr_setup.hpp"

class ActionAgentFixture : public dsr_util::ActionAgent<test_msgs::action::Fibonacci>
{
public:
  explicit ActionAgentFixture(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : dsr_util::ActionAgent<test_msgs::action::Fibonacci>("action_agent", options)
  {
  }

  bool get_goal_from_dsr(DSR::Node)
  {
    return return_state_;
  }

  void set_return_goal_from_dsr(bool rtn)
  {
    return_state_ = rtn;
  }

  rclcpp_action::ClientGoalHandle<test_msgs::action::Fibonacci>::WrappedResult get_result()
  {
    return result_;
  }

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }

  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    return dsr_util::NodeAgent::add_node<node_type>(name);
  }

  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & parent, const std::string & child)
  {
    return dsr_util::NodeAgent::add_edge<edge_type>(parent, child);
  }

private:
  bool return_state_{true};
};


class DummyActionServer : public rclcpp::Node
{
public:
  using ActionT = test_msgs::action::Fibonacci;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  DummyActionServer()
  : Node("dummy_server")
  {
    action_server_ = std::make_shared<ActionServer>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "test_fibonacci", std::bind(&DummyActionServer::executeCallback, this),
      nullptr, std::chrono::milliseconds(500), true);

    action_server_->activate();
  }

  ~DummyActionServer() = default;

  void setExecutionDuration(const std::chrono::milliseconds duration)
  {
    execution_duration_ = duration;
  }

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

    action_server_->publish_feedback(std::make_shared<typename ActionT::Feedback>());

    std::this_thread::sleep_for(execution_duration_);

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
  std::chrono::milliseconds execution_duration_{0};
};

TEST_F(DsrUtilTest, actionAgentConfigure) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_agent->activate();
  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
}

TEST_F(DsrUtilTest, actionAgentConfigureWithoutServer) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node_agent->shutdown();
  rclcpp::shutdown();
}

TEST_F(DsrUtilTest, actionAgentConfigureWithoutROSActionName) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("test_topic"));

  const auto state_after_configure = node_agent->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
}

TEST_F(DsrUtilTest, actionAgentAbortCancelAction) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("move"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Insert the nodes and an abort edge in the graph
  auto robot_node = node_agent->add_node<robot_node_type>("robot");
  auto action_node = node_agent->add_node<move_node_type>("move");
  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  auto edge = node_agent->add_edge<abort_edge_type>("robot", "move");

  // Check the result: the node should be removed
  EXPECT_FALSE(node_agent->get_graph()->get_node("move").has_value());

  // Now, we insert the action node again and a cancel edge
  action_node = node_agent->add_node<move_node_type>("move");
  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  edge = node_agent->add_edge<cancel_edge_type>("robot", "move");

  // Check the result: the node should be removed
  EXPECT_FALSE(node_agent->get_graph()->get_node("move").has_value());

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
}

TEST_F(DsrUtilTest, actionAgentWantsToActionSucceeded) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  dummy_server_node->setExecutionDuration(std::chrono::milliseconds(5));
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("move"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a thread to spin the node
  auto node_thread = std::thread([&]() {rclcpp::spin(node_agent->get_node_base_interface());});

  // Insert the nodes and a wants_to edge in the graph
  auto robot_node = node_agent->add_node<robot_node_type>("robot");
  auto action_node = node_agent->add_node<move_node_type>("move");

  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  auto edge = node_agent->add_edge<wants_to_edge_type>("robot", "move");

  // Wait until the edge becomes 'is_performing'
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  auto is_performing_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "is_performing");
  EXPECT_TRUE(is_performing_edge.has_value());

  // Wait for the action to finish
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  auto finished_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "finished");
  EXPECT_TRUE(finished_edge.has_value());

  // Check the result
  EXPECT_TRUE(node_agent->get_result().code == rclcpp_action::ResultCode::SUCCEEDED);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  node_thread.join();
}

TEST_F(DsrUtilTest, actionAgentWantsToActionAbort) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  dummy_server_node->setReturn(false);
  dummy_server_node->setExecutionDuration(std::chrono::milliseconds(5));
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("move"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a thread to spin the node
  auto node_thread = std::thread([&]() {rclcpp::spin(node_agent->get_node_base_interface());});

  // Insert the nodes and a wants_to edge in the graph
  auto robot_node = node_agent->add_node<robot_node_type>("robot");
  auto action_node = node_agent->add_node<move_node_type>("move");

  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  auto edge = node_agent->add_edge<wants_to_edge_type>("robot", "move");

  // Wait until the edge becomes 'is_performing'
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  auto is_performing_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "is_performing");
  EXPECT_TRUE(is_performing_edge.has_value());

  // Wait for the action to finish
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  auto failed_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "failed");
  EXPECT_TRUE(failed_edge.has_value());

  // Check the result
  EXPECT_TRUE(node_agent->get_result().code == rclcpp_action::ResultCode::ABORTED);

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  node_thread.join();
}

TEST_F(DsrUtilTest, actionAgentWantsToActionCancel) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  dummy_server_node->setExecutionDuration(std::chrono::milliseconds(5));
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("move"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Create a thread to spin the node
  auto node_thread = std::thread([&]() {rclcpp::spin(node_agent->get_node_base_interface());});

  // Insert the nodes and a wants_to edge in the graph
  auto robot_node = node_agent->add_node<robot_node_type>("robot");
  auto action_node = node_agent->add_node<move_node_type>("move");

  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  auto edge = node_agent->add_edge<wants_to_edge_type>("robot", "move");

  // Wait until the edge becomes 'is_performing'
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  auto is_performing_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "is_performing");
  EXPECT_TRUE(is_performing_edge.has_value());

  // Now insert a cancel edge
  edge = node_agent->add_edge<cancel_edge_type>("robot", "move");

  // Wait for the action to finish
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Check the result
  EXPECT_FALSE(node_agent->get_graph()->get_node("move").has_value());

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
  // Have to join thread after rclcpp is shut down otherwise test hangs.
  node_thread.join();
}

TEST_F(DsrUtilTest, actionAgentWantsToActionMissing) {
  rclcpp::init(0, nullptr);
  auto node_agent = std::make_shared<ActionAgentFixture>();
  auto dummy_server_node = std::make_shared<DummyActionServer>();
  node_agent->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  node_agent->declare_parameter("dsr_action_name", rclcpp::ParameterValue("move"));
  node_agent->declare_parameter("ros_action_name", rclcpp::ParameterValue("test_fibonacci"));

  // Configure and activate the node
  node_agent->configure();
  node_agent->activate();

  // Set the return value to false
  node_agent->set_return_goal_from_dsr(false);

  // Insert the nodes and a wants_to edge in the graph
  auto robot_node = node_agent->add_node<robot_node_type>("robot");
  auto action_node = node_agent->add_node<move_node_type>("move");

  EXPECT_TRUE(node_agent->get_graph()->get_node("move").has_value());

  auto edge = node_agent->add_edge<wants_to_edge_type>("robot", "move");

  // Wait until the edge becomes 'failed'
  auto failed_edge = node_agent->get_graph()->get_edge(
    node_agent->get_graph()->get_node("robot").value().id(),
    node_agent->get_graph()->get_node("move").value().id(), "failed");
  EXPECT_TRUE(failed_edge.has_value());

  node_agent->deactivate();
  node_agent->cleanup();
  node_agent->shutdown();
  dummy_server_node.reset();
  rclcpp::shutdown();
}
