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
#include "dsr_agents/tf_agent.hpp"
#include "dsr_util/utils/test_dsr_setup.hpp"
#include "lifecycle_msgs/msg/state.hpp"

class TFAgentFixture : public dsr_agents::TFAgent
{
public:
  TFAgentFixture()
  : dsr_agents::TFAgent()
  {
  }

  ~TFAgentFixture() = default;

  std::shared_ptr<dsr_util::DSRGraphExt> get_graph()
  {
    return G_;
  }

  void set_source(const std::string & source)
  {
    source_ = source;
  }

  tf2_msgs::msg::TFMessage sort_tf_by_parent_frame(tf2_msgs::msg::TFMessage & unsorted_trf)
  {
    return dsr_agents::TFAgent::sort_tf_by_parent_frame(unsorted_trf);
  }

  void replace_frames_with_dsr_names(tf2_msgs::msg::TFMessage & sorted_trf)
  {
    return dsr_agents::TFAgent::replace_frames_with_dsr_names(sorted_trf);
  }

  void insert_and_update_tf_into_dsr(const tf2_msgs::msg::TFMessage & sorted_trf)
  {
    return dsr_agents::TFAgent::insert_and_update_tf_into_dsr(sorted_trf);
  }
};

TEST_F(DsrUtilTest, tfAgentConfigure) {
  // Create the node
  auto agent_node = std::make_shared<TFAgentFixture>();
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));

  const auto state_after_configure = agent_node->configure();
  ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  agent_node->activate();
  agent_node->deactivate();
  agent_node->cleanup();
  agent_node->shutdown();
}

TEST_F(DsrUtilTest, tfAgentSortTFByParentFrame) {
  // Create the node
  auto agent_node = std::make_shared<TFAgentFixture>();

  // Create a sample unsorted TFMessage
  tf2_msgs::msg::TFMessage unsorted_trf;
  geometry_msgs::msg::TransformStamped trf1, trf2, trf3;

  trf1.header.frame_id = "map";
  trf1.child_frame_id = "base_link";
  trf2.header.frame_id = "base_link";
  trf2.child_frame_id = "camera_link";
  trf3.header.frame_id = "map";
  trf3.child_frame_id = "odom";

  unsorted_trf.transforms.push_back(trf1);
  unsorted_trf.transforms.push_back(trf2);
  unsorted_trf.transforms.push_back(trf3);

  // Sort the TFMessage
  auto sorted_trf = agent_node->sort_tf_by_parent_frame(unsorted_trf);

  // Check the order of the sorted TFMessage
  ASSERT_EQ(sorted_trf.transforms.size(), 3);
  EXPECT_EQ(sorted_trf.transforms[0].header.frame_id, "map");
  EXPECT_EQ(sorted_trf.transforms[0].child_frame_id, "base_link");
  EXPECT_EQ(sorted_trf.transforms[1].header.frame_id, "map");
  EXPECT_EQ(sorted_trf.transforms[1].child_frame_id, "odom");
  EXPECT_EQ(sorted_trf.transforms[2].header.frame_id, "base_link");
  EXPECT_EQ(sorted_trf.transforms[2].child_frame_id, "camera_link");
}

TEST_F(DsrUtilTest, tfAgentReplaceFramesWithDSRNames) {
  // Create the node
  auto agent_node = std::make_shared<TFAgentFixture>();

  // Set the source
  agent_node->set_source("robot");

  // Create a sample sorted TFMessage
  tf2_msgs::msg::TFMessage sorted_trf;
  geometry_msgs::msg::TransformStamped trf1, trf2, trf3;

  trf1.header.frame_id = "map";
  trf1.child_frame_id = "base_link";
  trf2.header.frame_id = "base_link";
  trf2.child_frame_id = "map";
  trf3.header.frame_id = "camera_link";
  trf3.child_frame_id = "odom";

  sorted_trf.transforms.push_back(trf1);
  sorted_trf.transforms.push_back(trf2);
  sorted_trf.transforms.push_back(trf3);

  // Replace the frames with the DSR names
  agent_node->replace_frames_with_dsr_names(sorted_trf);

  // Check the DSR names
  ASSERT_EQ(sorted_trf.transforms.size(), 3);
  EXPECT_EQ(sorted_trf.transforms[0].header.frame_id, "world");
  EXPECT_EQ(sorted_trf.transforms[0].child_frame_id, "robot");
  EXPECT_EQ(sorted_trf.transforms[1].header.frame_id, "robot");
  EXPECT_EQ(sorted_trf.transforms[1].child_frame_id, "world");
  EXPECT_EQ(sorted_trf.transforms[2].header.frame_id, "camera_link");
  EXPECT_EQ(sorted_trf.transforms[2].child_frame_id, "odom");
}

TEST_F(DsrUtilTest, tfAgentInsertAndUpdateTFIntoDSR) {
  // Create the node
  auto agent_node = std::make_shared<TFAgentFixture>();
  agent_node->declare_parameter("dsr_input_file", rclcpp::ParameterValue(test_file_));
  agent_node->declare_parameter("source", rclcpp::ParameterValue("test_robot"));

  // Configure and activate the node
  agent_node->configure();
  agent_node->activate();

  // Create a sample sorted TFMessage
  tf2_msgs::msg::TFMessage sorted_trf;
  geometry_msgs::msg::TransformStamped trf1, trf2, trf3;

  trf1.header.frame_id = "map";
  trf1.child_frame_id = "base_link";
  trf1.transform.translation.x = 1.0;
  trf1.transform.translation.y = 2.0;
  trf1.transform.translation.z = 3.0;
  trf1.transform.rotation.x = 0.0;
  trf1.transform.rotation.y = 0.0;
  trf1.transform.rotation.z = 0.0;
  trf1.transform.rotation.w = 1.0;

  trf2.header.frame_id = "base_link";
  trf2.child_frame_id = "camera_link";
  trf2.transform.translation.x = 4.0;
  trf2.transform.translation.y = 5.0;
  trf2.transform.translation.z = 6.0;
  trf2.transform.rotation.x = 0.0;
  trf2.transform.rotation.y = 0.0;
  trf2.transform.rotation.z = 0.0;
  trf2.transform.rotation.w = 1.0;

  trf3.header.frame_id = "map";
  trf3.child_frame_id = "odom";
  trf3.transform.translation.x = 7.0;
  trf3.transform.translation.y = 8.0;
  trf3.transform.translation.z = 9.0;
  trf3.transform.rotation.x = 0.0;
  trf3.transform.rotation.y = 0.0;
  trf3.transform.rotation.z = 0.0;
  trf3.transform.rotation.w = 1.0;

  sorted_trf.transforms.push_back(trf1);
  sorted_trf.transforms.push_back(trf2);
  sorted_trf.transforms.push_back(trf3);

  // Insert and update the TFMessage into the DSR graph
  agent_node->insert_and_update_tf_into_dsr(sorted_trf);

  // Check the nodes in the DSR graph
  EXPECT_TRUE(agent_node->get_graph()->get_node("world").has_value());
  EXPECT_TRUE(agent_node->get_graph()->get_node("map").has_value());
  EXPECT_TRUE(agent_node->get_graph()->get_node("base_link").has_value());
  EXPECT_TRUE(agent_node->get_graph()->get_node("camera_link").has_value());
  EXPECT_TRUE(agent_node->get_graph()->get_node("odom").has_value());

  // Check the edges and RT attributes
  auto rt_edge1 = agent_node->get_graph()->get_edge(
    agent_node->get_graph()->get_node("map").value().id(),
    agent_node->get_graph()->get_node("base_link").value().id(), "RT");
  EXPECT_TRUE(rt_edge1.has_value());

  auto attrs1 = rt_edge1.value().attrs();
  auto trans1 = std::get<std::vector<float>>(attrs1["rt_translation"].value());
  EXPECT_FLOAT_EQ(trans1[0], 1.0);
  EXPECT_FLOAT_EQ(trans1[1], 2.0);
  EXPECT_FLOAT_EQ(trans1[2], 3.0);

  auto rot1 = std::get<std::vector<float>>(attrs1["rt_rotation_euler_xyz"].value());
  EXPECT_FLOAT_EQ(rot1[0], 0.0);
  EXPECT_FLOAT_EQ(rot1[1], 0.0);
  EXPECT_FLOAT_EQ(rot1[2], 0.0);

  auto rt_edge2 = agent_node->get_graph()->get_edge(
    agent_node->get_graph()->get_node("base_link").value().id(),
    agent_node->get_graph()->get_node("camera_link").value().id(), "RT");
  EXPECT_TRUE(rt_edge2.has_value());

  auto attrs2 = rt_edge2.value().attrs();
  auto trans2 = std::get<std::vector<float>>(attrs2["rt_translation"].value());
  EXPECT_FLOAT_EQ(trans2[0], 4.0);
  EXPECT_FLOAT_EQ(trans2[1], 5.0);
  EXPECT_FLOAT_EQ(trans2[2], 6.0);

  auto rot2 = std::get<std::vector<float>>(attrs2["rt_rotation_euler_xyz"].value());
  EXPECT_FLOAT_EQ(rot2[0], 0.0);
  EXPECT_FLOAT_EQ(rot2[1], 0.0);
  EXPECT_FLOAT_EQ(rot2[2], 0.0);

  auto rt_edge3 = agent_node->get_graph()->get_edge(
    agent_node->get_graph()->get_node("map").value().id(),
    agent_node->get_graph()->get_node("odom").value().id(), "RT");
  EXPECT_TRUE(rt_edge3.has_value());

  auto attrs3 = rt_edge3.value().attrs();
  auto trans3 = std::get<std::vector<float>>(attrs3["rt_translation"].value());
  EXPECT_FLOAT_EQ(trans3[0], 7.0);
  EXPECT_FLOAT_EQ(trans3[1], 8.0);
  EXPECT_FLOAT_EQ(trans3[2], 9.0);

  auto rot3 = std::get<std::vector<float>>(attrs3["rt_rotation_euler_xyz"].value());
  EXPECT_FLOAT_EQ(rot3[0], 0.0);
  EXPECT_FLOAT_EQ(rot3[1], 0.0);
  EXPECT_FLOAT_EQ(rot3[2], 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
