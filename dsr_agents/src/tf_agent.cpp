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

// DSR
#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/tf_agent.hpp"

namespace dsr_agents
{

TFAgent::TFAgent(const rclcpp::NodeOptions & options)
: dsr_util::AgentNode("tf_agent", options)
{
}

dsr_util::CallbackReturn TFAgent::on_configure(const rclcpp_lifecycle::State & state)
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

  return AgentNode::on_configure(state);
}

dsr_util::CallbackReturn TFAgent::on_cleanup(const rclcpp_lifecycle::State & state)
{
  // Cleaning the subscribers
  tf_sub_.reset();
  tf_static_sub_.reset();

  // Delete all the nodes from the DSR graph
  for (auto dsr_name : dsr_nodes_) {
    if (auto dsr_node = G_->get_node(dsr_name); dsr_node.has_value()) {
      delete_node(dsr_name);
    }
  }

  return AgentNode::on_cleanup(state);
}

void TFAgent::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // Sort the transforms
  tf2_msgs::msg::TFMessage sorted_trf = sort_tf_by_parent_frame(*msg);

  // Replace 'base_link' with source_ and 'map' with world
  replace_frames_with_dsr_names(sorted_trf);

  // Store the names of the nodes in a vector
  store_dsr_names(sorted_trf, dsr_nodes_);

  // Insert the transforms into the DSR graph
  insert_and_update_tf_into_dsr(sorted_trf);
}

tf2_msgs::msg::TFMessage TFAgent::sort_tf_by_parent_frame(tf2_msgs::msg::TFMessage & unsorted_trf)
{
  tf2_msgs::msg::TFMessage sorted_trf;
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

  return sorted_trf;
}

void TFAgent::replace_frames_with_dsr_names(tf2_msgs::msg::TFMessage & sorted_trf)
{
  for (auto & trf : sorted_trf.transforms) {
    trf.header.frame_id = (trf.header.frame_id == "base_link") ? source_ : trf.header.frame_id;
    trf.header.frame_id = (trf.header.frame_id == "map") ? "world" : trf.header.frame_id;
    trf.child_frame_id = (trf.child_frame_id == "base_link") ? source_ : trf.child_frame_id;
    trf.child_frame_id = (trf.child_frame_id == "map") ? "world" : trf.child_frame_id;
  }
}

void TFAgent::insert_and_update_tf_into_dsr(const tf2_msgs::msg::TFMessage & sorted_trf)
{
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

void TFAgent::store_dsr_names(
  const tf2_msgs::msg::TFMessage & sorted_trf, std::vector<std::string> & dsr_names)
{
  for (auto trf : sorted_trf.transforms) {
    dsr_names.push_back(trf.header.frame_id);
    dsr_names.push_back(trf.child_frame_id);
  }

  // Remove duplicates
  std::sort(dsr_names.begin(), dsr_names.end());
  dsr_names.erase(std::unique(dsr_names.begin(), dsr_names.end()), dsr_names.end());
}

}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::TFAgent)
