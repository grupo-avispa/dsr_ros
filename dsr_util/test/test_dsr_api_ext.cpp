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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "dsr_util/dsr_api_ext.hpp"

class DSRGraphExtFixture : public dsr_util::DSRGraphExt
{
public:
  DSRGraphExtFixture(std::string name, uint32_t agent_identifier, std::string input_file)
  : dsr_util::DSRGraphExt(name, agent_identifier, input_file)
  {
  }
  ~DSRGraphExtFixture() = default;

  std::tuple<float, float> get_position_by_level_in_graph(const DSR::Node & parent)
  {
    return dsr_util::DSRGraphExt::get_position_by_level_in_graph(parent);
  }

  std::tuple<float, float> get_random_position_to_draw_in_graph()
  {
    return dsr_util::DSRGraphExt::get_random_position_to_draw_in_graph();
  }
};

class DSRApiExtTest : public ::testing::Test
{
public:
  DSRApiExtTest() = default;
  ~DSRApiExtTest() = default;

  void SetUp() override
  {
    auto pkg = ament_index_cpp::get_package_share_directory("dsr_util");
    G_ = std::make_shared<DSRGraphExtFixture>("test", 2, pkg + "/test/test_dsr.json");
  }

  void TearDown() override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    G_.reset();
  }

protected:
  std::shared_ptr<DSRGraphExtFixture> G_;
};

TEST_F(DSRApiExtTest, createNodeWithPriority) {
  auto node = G_->create_node_with_priority<robot_node_type>("test_node", 5, "test_source");

  EXPECT_EQ(node.name(), "test_node");
  EXPECT_EQ(node.type(), "robot");

  auto attributes = node.attrs();
  auto search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 0);

  // Just a value different from 0
  search = attributes.find("pos_x");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_NE(std::get<float>(search->second.value()), 0);
  search = attributes.find("pos_y");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_NE(std::get<float>(search->second.value()), 0);

  // Check priority and source attributes using the get functions
  EXPECT_EQ(G_->get_priority(node), 5);
  EXPECT_EQ(G_->get_source(node), "test_source");
}

TEST_F(DSRApiExtTest, createNodeWithPose) {
  auto parent_node =
    G_->create_node_with_priority<robot_node_type>("parent_node", 0, "test_source");

  if (auto id = G_->insert_node(parent_node); id.has_value()) {
    auto child_node = G_->create_node_with_pose<robot_node_type, RT_edge_type>(
      "test_node", "parent_node", 0, "test_source");

    EXPECT_EQ(child_node.name(), "test_node");
    EXPECT_EQ(child_node.type(), "robot");

    // Check parent and level attributes, not the random position
    auto attributes = child_node.attrs();
    auto search = attributes.find("parent");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_EQ(std::get<uint64_t>(search->second.value()), id.value());

    search = attributes.find("level");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_EQ(std::get<int>(search->second.value()), 1);

    search = attributes.find("pos_x");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_NE(std::get<float>(search->second.value()), 0);

    search = attributes.find("pos_y");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_NE(std::get<float>(search->second.value()), 0);
  }
}

TEST_F(DSRApiExtTest, createNodeWithPoseNotRT) {
  auto parent_node =
    G_->create_node_with_priority<robot_node_type>("parent_node", 0, "test_source");

  if (auto id = G_->insert_node(parent_node); id.has_value()) {
    auto child_node = G_->create_node_with_pose<robot_node_type, is_edge_type>(
      "test_node", "parent_node", 0, "test_source");

    EXPECT_EQ(child_node.name(), "test_node");
    EXPECT_EQ(child_node.type(), "robot");

    // Check parent and level attributes, not the random position
    auto attributes = child_node.attrs();
    auto search = attributes.find("parent");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_EQ(std::get<uint64_t>(search->second.value()), id.value());

    search = attributes.find("level");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_EQ(std::get<int>(search->second.value()), 1);

    search = attributes.find("pos_x");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_NE(std::get<float>(search->second.value()), 0);

    search = attributes.find("pos_y");
    EXPECT_TRUE(search != attributes.end());
    EXPECT_NE(std::get<float>(search->second.value()), 0);
  }
}

TEST_F(DSRApiExtTest, createNodeWithPoseNotRelative) {
  auto child_node = G_->create_node_with_pose<robot_node_type, RT_edge_type>(
    "test_node", "parent_node", 0, "test_source");

  EXPECT_EQ(child_node.name(), "test_node");
  EXPECT_EQ(child_node.type(), "robot");

  // Check parent and level attributes, not the random position
  auto attributes = child_node.attrs();
  auto search = attributes.find("parent");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<uint64_t>(search->second.value()), 0);

  search = attributes.find("level");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<int>(search->second.value()), 0);

  search = attributes.find("pos_x");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_NE(std::get<float>(search->second.value()), 0);

  search = attributes.find("pos_y");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_NE(std::get<float>(search->second.value()), 0);
}

TEST_F(DSRApiExtTest, createEdgeWithSource) {
  auto parent_node =
    G_->create_node_with_priority<robot_node_type>("parent_node", 6, "test_source");
  auto child_node =
    G_->create_node_with_priority<robot_node_type>("child_node", 8, "test_source");
  auto edge = G_->create_edge_with_source<is_edge_type>(
    parent_node.id(), child_node.id(), "test_source");

  EXPECT_EQ(edge.type(), "is");
  EXPECT_EQ(edge.from(), parent_node.id());
  EXPECT_EQ(edge.to(), child_node.id());

  auto attributes = edge.attrs();
  auto search = attributes.find("source");
  EXPECT_TRUE(search != attributes.end());
  EXPECT_EQ(std::get<std::string>(search->second.value()), "test_source");
}

TEST_F(DSRApiExtTest, getPriority) {
  auto node = G_->create_node_with_priority<robot_node_type>("test_node", 5, "test_source");
  EXPECT_EQ(G_->get_priority(node), 5);

  auto node2 = DSR::Node::create<robot_node_type>("test_node2");
  EXPECT_EQ(G_->get_priority(node2), std::numeric_limits<int>::quiet_NaN());
}

TEST_F(DSRApiExtTest, get_source) {
  auto node = G_->create_node_with_priority<robot_node_type>("test_node", 5, "test_source");
  EXPECT_EQ(G_->get_source(node), "test_source");

  auto node2 = DSR::Node::create<robot_node_type>("test_node2");
  EXPECT_EQ(G_->get_source(node2), std::string());
}

TEST_F(DSRApiExtTest, getPositionByLevelInGraph) {
  // Insert a parent and a child node in the graph
  auto parent_node =
    G_->create_node_with_priority<robot_node_type>("parent_node", 0, "test_source");

  if (auto id = G_->insert_node(parent_node); id.has_value()) {
    auto first_child_node =
      G_->create_node_with_priority<robot_node_type>("first_child_node", 8, "test_source");
    if (auto id2 = G_->insert_node(first_child_node); id2.has_value()) {
      // Create a RT edge between the parent and the child and insert it in the graph
      auto edge = G_->create_edge_with_source<RT_edge_type>(id.value(), id2.value(), "test_source");
      if (G_->insert_or_assign_edge(edge)) {
        // Get the position of the child node in the graph
        const auto &[pos_x, pos_y] = G_->get_position_by_level_in_graph(parent_node);
        float maxx = G_->get_attrib_by_name<pos_x_att>(parent_node).value() - 300;
        float maxy = G_->get_attrib_by_name<pos_y_att>(parent_node).value();
        EXPECT_NE(pos_x, maxx - 200);
        EXPECT_NE(pos_y, maxy - 80);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
