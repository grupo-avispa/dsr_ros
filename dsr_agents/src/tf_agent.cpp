// Copyright (c) 2023 Alberto J. Tudela Roldán
// Copyright (c) 2023 Grupo Avispa, DTE, Universidad de Málaga
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

// ROS
#include "nav2_util/node_utils.hpp"

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/tf_agent.hpp"

namespace dsr_agents
{

TFAgent::TFAgent(const rclcpp::NodeOptions & options)
: dsr_util::AgentNode("tf_agent", options)
{
  // Subscriber to the tf topics
  auto latched_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf",
    rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    std::bind(&TFAgent::tf_callback, this, std::placeholders::_1));
  tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    latched_profile,
    std::bind(&TFAgent::tf_callback, this, std::placeholders::_1));
}

void TFAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  tf2_msgs::msg::TFMessage unsorted_trf, sorted_trf;
  unsorted_trf = *msg;

  // Sort the transforms
  int i = 0;
  std::string parent_frame = unsorted_trf.transforms[0].header.frame_id;
  do {
    // Copy the transforms from the unsorted vector
    // to the sorted vector if the parent frame is the same
    std::copy_if(
      unsorted_trf.transforms.begin(), unsorted_trf.transforms.end(),
      std::back_inserter(sorted_trf.transforms),
      [parent_frame](const auto & trf) {
        return trf.header.frame_id == parent_frame;
      });
    // Erase the elements from the original vector
    unsorted_trf.transforms.erase(
      std::remove_if(
        unsorted_trf.transforms.begin(),
        unsorted_trf.transforms.end(),
        [parent_frame](const auto & trf) {
          return trf.header.frame_id == parent_frame;
        }),
      unsorted_trf.transforms.end());
    // Get the new parent frame
    parent_frame = sorted_trf.transforms[i].child_frame_id;
    i++;
  } while (unsorted_trf.transforms.size() > 0);

  // Replace 'base_link' with robot and 'map' with world
  std::for_each(
    sorted_trf.transforms.begin(), sorted_trf.transforms.end(),
    [source = source_](auto & trf) {
      trf.header.frame_id = (trf.header.frame_id == "base_link") ? source : trf.header.frame_id;
      trf.header.frame_id = (trf.header.frame_id == "map") ? "world" : trf.header.frame_id;
      trf.child_frame_id = (trf.child_frame_id == "base_link") ? source : trf.child_frame_id;
      trf.child_frame_id = (trf.child_frame_id == "map") ? "world" : trf.child_frame_id;
    });

  for (auto trf : sorted_trf.transforms) {
    // Get the parent and child frames
    std::string new_parent_frame = trf.header.frame_id;
    std::string new_child_frame = trf.child_frame_id;

    // Get the nodes
    std::optional<DSR::Node> parent_node = G_->get_node(new_parent_frame);
    std::optional<DSR::Node> child_node = G_->get_node(new_child_frame);

    // Create the parent node
    if (!parent_node.has_value()) {
      add_node_with_edge<transform_node_type, RT_edge_type>(new_parent_frame, "");
    }

    // Create the child node
    if (!child_node.has_value()) {
      add_node_with_edge<transform_node_type, RT_edge_type>(new_child_frame, new_parent_frame);
    }

    // Update the RT attributes
    if (auto new_parent_node = G_->get_node(new_parent_frame); new_parent_node.has_value()) {
      if (auto new_child_node = G_->get_node(new_child_frame); new_child_node.has_value()) {
        update_rt_attributes(new_parent_node.value(), new_child_node.value(), trf.transform);
        RCLCPP_DEBUG(
          this->get_logger(),
          "Update edge [%s] -> [%s]", new_parent_frame.c_str(), new_child_frame.c_str());
      }
    }
  }
}

}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::TFAgent)
