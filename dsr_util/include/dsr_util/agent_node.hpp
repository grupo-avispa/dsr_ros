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

#ifndef DSR_UTIL__AGENT_NODE_HPP_
#define DSR_UTIL__AGENT_NODE_HPP_

// Qt
#include <QObject>

// C++
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

// ROS
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr_msgs/srv/save_dsr.hpp"
#include "dsr_util/dsr_api_ext.hpp"

namespace dsr_util
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class dsr_util::AgentNode
 * @brief Base class to connect the DSR graph with ROS 2. It contains common methods and attributes
 * to send data from ROS 2 to the DSR graph and vice versa. All agents must inherit from this class.
 */
class AgentNode : public QObject, public rclcpp_lifecycle::LifecycleNode
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Agent Node object.
   *
   * @param ros_node_name Name of the ROS node and the DSR agent.
   * @param options Node options
   */
  explicit AgentNode(
    std::string ros_node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Destroy the Agent Node object.
   *
   */
  virtual ~AgentNode() = default;

protected:
  /**
   * @brief Add a node into the DSR graph with the given name and type.
   * By default, all nodes have a low priority (0) and the source attribute is set
   * to the name of the physical machine.
   *
   * @tparam node_type The type of the DSR node. Defined in ros_to_dsr_types.hpp.
   * @param name Name of the DSR node.
   * @return std::optional<DSR::Node> The DSR node if it was added successfully,
   */
  template<typename node_type>
  std::optional<DSR::Node> add_node(const std::string & name)
  {
    std::optional<DSR::Node> return_node;
    // Create the node
    auto new_node =
      G_->create_node_with_pose<node_type, is_edge_type>(name, "", 0, source_);
    // Insert the node into the DSR graph
    if (auto id = G_->insert_node(new_node); id.has_value()) {
      return_node = new_node;
      RCLCPP_INFO(
        this->get_logger(),
        "Inserted node [%s] successfully of type [%s]", name.c_str(), new_node.type().c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "The node [%s] couldn't be inserted", name.c_str());
    }
    return return_node;
  }

  /**
   * @brief Add a node with an edge into the DSR graph with the given name,
   * the name of connecting node and the direction of the edge.
   * By default, all nodes have a low priority (0) and the source attribute is set
   * to the name of the physical machine.
   *
   * @tparam node_type The type of the DSR node. Defined in ros_to_dsr_types.hpp.
   * @tparam edge_type The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
   * @param name Name of the DSR node.
   * @param connecting_node_name Name of the connected DSR node.
   * @param reversed If the new_node is connected to the connecting_node,
   * false if the connecting_node is connected to the new_node. Default is false. Example:
   *    connecting_node --> new_node if reversed is false
   *    new_node --> connecting_node if reversed is true
   * @return std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
   * The DSR node and the DSR edge if they were added successfully.
   */
  template<typename node_type, typename edge_type>
  std::tuple<std::optional<DSR::Node>, std::optional<DSR::Edge>>
  add_node_with_edge(
    const std::string & name, const std::string & connecting_node_name, const bool reversed = false)
  {
    std::optional<DSR::Node> return_node;
    std::optional<DSR::Edge> return_edge;
    // Create the node
    auto new_node =
      G_->create_node_with_pose<node_type, edge_type>(name, connecting_node_name, 0, source_);
    // Insert the node into the DSR graph
    if (auto id = G_->insert_node(new_node); id.has_value()) {
      return_node = new_node;
      RCLCPP_INFO(
        this->get_logger(),
        "Inserted node [%s] successfully of type [%s]", name.c_str(), new_node.type().c_str());
      // Insert the edge into the DSR graph
      std::string parent_name = reversed ? name : connecting_node_name;
      std::string child_name = reversed ? connecting_node_name : name;
      return_edge = add_edge<edge_type>(parent_name, child_name);
    } else {
      RCLCPP_ERROR(this->get_logger(), "The node [%s] couldn't be inserted", name.c_str());
    }
    return std::make_tuple(return_node, return_edge);
  }

  /**
   * @brief Add an edge into the DSR graph with the given parent and child nodes names.
   * By default, all edges have the source attribute set to the name of the physical machine.
   *
   * @tparam edge_type The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
   * @param from Name of the parent DSR node.
   * @param to  Name of the child DSR node.
   * @return std::optional<DSR::Edge> The DSR edge if it was added successfully.
   */
  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(const std::string & from, const std::string & to)
  {
    std::optional<DSR::Edge> return_edge;
    // Get the relatives nodes
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    // Insert the edge into the DSR graph
    if (parent_node.has_value()) {
      if (child_node.has_value()) {
        // Create the edge
        auto new_edge = G_->create_edge_with_source<edge_type>(
          parent_node.value().id(), child_node.value().id(), source_);
        // Insert the edge into the DSR graph
        if (G_->insert_or_assign_edge(new_edge)) {
          return_edge = new_edge;
          RCLCPP_INFO(
            this->get_logger(),
            "Inserted edge [%s->%s] successfully of type [%s]",
            parent_node.value().name().c_str(), child_node.value().name().c_str(),
            new_edge.type().c_str());
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "The edge [%s->%s] of type [%s] couldn't be inserted",
            parent_node.value().name().c_str(), child_node.value().name().c_str(),
            new_edge.type().c_str());
        }
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "The edge couldn't be inserted because the child node [%s] doesn't exists", to.c_str());
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "The edge couldn't be inserted because the parent node [%s] doesn't exists", from.c_str());
    }
    return return_edge;
  }

  /**
   * @brief Add an edge into the DSR graph with the given parent and child nodes id.
   * By default, all edges have the source attribute set to the name of the physical machine.
   *
   * @tparam edge_type The type of the DSR edge. Defined in ros_to_dsr_types.hpp.
   * @param from Id of the parent DSR node.
   * @param to  Id of the child DSR node.
   * @return std::optional<DSR::Edge> The DSR edge if it was added successfully.
   */
  template<typename edge_type>
  std::optional<DSR::Edge> add_edge(uint64_t from, uint64_t to)
  {
    std::optional<DSR::Edge> return_edge;
    // Get the relatives nodes
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    // Insert the edge into the DSR graph
    if (parent_node.has_value() && child_node.has_value()) {
      // Create the edge
      auto new_edge = G_->create_edge_with_source<edge_type>(from, to, source_);
      // Insert the edge into the DSR graph
      if (G_->insert_or_assign_edge(new_edge)) {
        return_edge = new_edge;
        RCLCPP_INFO(
          this->get_logger(),
          "Inserted edge [%s->%s] successfully of type [%s]",
          parent_node.value().name().c_str(), child_node.value().name().c_str(),
          new_edge.type().c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "The edge [%s->%s] of type [%s] couldn't be inserted",
          parent_node.value().name().c_str(), child_node.value().name().c_str(),
          new_edge.type().c_str());
      }
    }
    return return_edge;
  }

  /**
   * @brief Delete a node into the DSR graph with the given id.
   * This method previously checks if the node exists.
   *
   * @param id Id of the DSR node.
   * @param to Id of the child DSR node.
   * @return true If the node was deleted successfully. False otherwise.
   */
  bool delete_node(uint64_t id)
  {
    // Check if the node exists
    bool success = false;
    if (auto node = G_->get_node(id); node.has_value()) {
      // Delete the node
      if (G_->delete_node(id)) {
        RCLCPP_INFO(
          this->get_logger(), "Deleted node [%s] successfully", node.value().name().c_str());
        success = true;
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "The node [%s] couldn't be deleted", node.value().name().c_str());
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(), "The node [%lu] couldn't be deleted because it doesn't exists", id);
    }
    return success;
  }

  /**
   * @brief Delete a node into the DSR graph with the given id.
   * This method previously checks if the node exists.
   *
   * @param id Id of the DSR node.
   * @param to Id of the child DSR node.
   * @return true If the node was deleted successfully. False otherwise.
   */
  bool delete_node(const std::string & name)
  {
    return delete_node(G_->get_node(name).value().id());
  }

  /**
   * @brief Delete an edge into the DSR graph with the given parent and child nodes id and
   * the edge type. This method previously checks if the parent and child nodes exist.
   *
   * @param from Id of the parent DSR node.
   * @param to Id of the child DSR node.
   * @param edge_type Name of the DSR edge.
   * @return true If the edge was replaced successfully. False otherwise.
   */
  bool delete_edge(uint64_t from, uint64_t to, std::string edge_type)
  {
    // Check if the parent and child nodes exist and if the edge exists
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    if (parent_node.has_value() && child_node.has_value()) {
      if (auto edge = G_->get_edge(from, to, edge_type); edge.has_value()) {
        // Delete the edge
        if (G_->delete_edge(from, to, edge_type)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Deleted edge [%s->%s] successfully of type [%s]",
            parent_node.value().name().c_str(), child_node.value().name().c_str(),
            edge_type.c_str());
          return true;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "The edge [%s->%s] of type [%s] couldn't be deleted",
            parent_node.value().name().c_str(), child_node.value().name().c_str(),
            edge_type.c_str());
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge [%lu->%lu] of type [%s] couldn't be deleted because it doesn't exists",
          from, to, edge_type.c_str());
      }
    }
    return false;
  }

  /**
   * @brief Delete an edge into the DSR graph with the given parent and child nodes names and
   * the edge type. This method previously checks if the parent and child nodes exist.
   *
   * @param from Name of the parent DSR node.
   * @param to Name of the child DSR node.
   * @param edge_type Name of the DSR edge.
   * @return true If the edge was replaced successfully. False otherwise.
   */
  bool delete_edge(const std::string & from, const std::string & to, std::string edge_type)
  {
    // Check if the parent and child nodes exist
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    if (parent_node.has_value()) {
      if (child_node.has_value()) {
        return delete_edge(parent_node.value().id(), child_node.value().id(), edge_type);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge couldn't be deleted because the child node [%s] doesn't exists", to.c_str());
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "The edge couldn't be deleted because the parent node [%s] doesn't exists", from.c_str());
    }
    return false;
  }

  /**
   * @brief Replace an edge into the DSR graph with the given parent and child nodes id and
   * the old edge type. This method previously checks if the parent and child nodes exist.
   * By default, all edges have the source attribute set to the name of the physical machine.
   *
   * @tparam edge_type The type of the new DSR edge. Defined in ros_to_dsr_types.hpp.
   * @param from Id of the parent DSR node.
   * @param to Id of the child DSR node.
   * @param old_edge Name of the old DSR edge.
   * @return true If the edge was replaced successfully. False otherwise.
   */
  template<typename edge_type>
  bool replace_edge(uint64_t from, uint64_t to, std::string old_edge)
  {
    // Check if the parent and child nodes exist and if the old edge exists
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    if (parent_node.has_value()) {
      if (child_node.has_value()) {
        if (auto edge = G_->get_edge(from, to, old_edge); edge.has_value()) {
          // Delete the old edge
          if (G_->delete_edge(from, to, old_edge)) {
            // Create the new edge
            auto new_edge = G_->create_edge_with_source<edge_type>(from, to, source_);
            // Insert the new edge into the DSR graph
            if (G_->insert_or_assign_edge(new_edge)) {
              RCLCPP_INFO(
                this->get_logger(),
                "Replaced edge [%s->%s] of type [%s] with type [%s]",
                parent_node.value().name().c_str(), child_node.value().name().c_str(),
                old_edge.c_str(), new_edge.type().c_str());
              return true;
            }
          } else {
            RCLCPP_ERROR(
              this->get_logger(),
              "The edge [%s->%s] of type [%s] couldn't be deleted",
              parent_node.value().name().c_str(), child_node.value().name().c_str(),
              old_edge.c_str());
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "The edge [%lu->%lu] of type [%s] couldn't be deleted "
            "because it doesn't exists", from, to, old_edge.c_str());
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "The edge [%lu->%lu] of type [%s] couldn't be deleted because "
          "the child node [%lu] doesn't exists", from, to, old_edge.c_str(), to);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "The edge [%lu->%lu] of type [%s] couldn't be deleted because "
        "the parent node [%lu] doesn't exists", from, to, old_edge.c_str(), from);
    }
    return false;
  }

  /**
   * @brief Replace an edge into the DSR graph with the given parent and child nodes names and
   * the old edge type. This method previously checks if the parent and child nodes exist.
   *
   * @tparam edge_type The type of the new DSR edge. Defined in ros_to_dsr_types.hpp.
   * @param from Name of the parent DSR node.
   * @param to Name of the child DSR node.
   * @param old_edge Name of the old DSR edge.
   * @return true If the edge was replaced successfully. False otherwise.
   */
  template<typename edge_type>
  bool replace_edge(const std::string & from, const std::string & to, std::string old_edge)
  {
    // Check if the parent and child nodes exist
    auto parent_node = G_->get_node(from);
    auto child_node = G_->get_node(to);
    if (parent_node.has_value() && child_node.has_value()) {
      return replace_edge<edge_type>(parent_node.value().id(), child_node.value().id(), old_edge);
    }
    return false;
  }

  /**
   * @brief Update the RT atributes (position and orientation) of the given node in the DSR graph
   * with the given ROS Transform message.
   *
   * @param from Parent DSR node.
   * @param to Child DSR node.
   * @param msg Transform message.
   */
  void update_rt_attributes(
    DSR::Node & from, DSR::Node & to, const geometry_msgs::msg::Transform & msg);

  /**
   * @brief Pointer to the DSR graph.
   */
  std::shared_ptr<dsr_util::DSRGraphExt> G_;

  /**
   * @brief Pointer to the RT API.
   */
  std::unique_ptr<DSR::RT_API> rt_;

  /**
   * @brief Name of the physical machine.
   */
  std::string source_;

/**
 * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
 *
 * @param node A node in which given parameter to be declared
 * @param param_name The name of parameter
 * @param default_value Parameter value to initialize with
 * @param parameter_descriptor Parameter descriptor (optional)
*/
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  /**
   * @brief Initialize ROS parameters.
   */
  void get_common_params();

private:
  /**
   * @brief Save the DSR graph into a file.
   *
   * @param request URL of the file.
   * @param response True if the DSR graph was saved successfully, false otherwise.
   */
  void save_dsr(
    const std::shared_ptr<dsr_msgs::srv::SaveDSR::Request> request,
    std::shared_ptr<dsr_msgs::srv::SaveDSR::Response> response);

  /**
   * @brief Callback executed when a node is updated in the DSR graph.
   *
   * @param id The id of the node.
   * @param type The type of the node.
   */
  virtual void node_updated(std::uint64_t /*id*/, const std::string & /*type*/) {}

  /**
   * @brief Callback executed when a node attribute is updated in the DSR graph.
   *
   * @param id The id of the node.
   * @param att_names The names of the attributes updated.
   */
  virtual void node_attr_updated(uint64_t /*id*/, const std::vector<std::string> & /*att_names*/) {}

  /**
   * @brief Callback executed when an edge is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   */
  virtual void edge_updated(
    std::uint64_t /*from*/, std::uint64_t /*to*/, const std::string & /*type*/) {}

  /**
   * @brief Callback executed when an edge attribute is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   * @param att_names The names of the attributes updated.
   */
  virtual void edge_attr_updated(
    std::uint64_t /*from*/, std::uint64_t /*to*/,
    const std::string & /*type*/, const std::vector<std::string> & /*att_names*/) {}

  /**
   * @brief Callback executed when a node is deleted in the DSR graph.
   *
   * @param id The id of the node.
   */
  virtual void node_deleted(std::uint64_t /*id*/) {}

  /**
   * @brief Callback executed when an edge is deleted in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param edge_tag The type of the edge.
   */
  virtual void edge_deleted(
    std::uint64_t /*from*/, std::uint64_t /*to*/, const std::string & /*edge_tag*/) {}

  // Service to save the DSR graph into a file.
  rclcpp::Service<dsr_msgs::srv::SaveDSR>::SharedPtr save_dsr_service_;

  // Id of the DSR agent.
  int agent_id_;

  // Name of the agent.
  std::string agent_name_;

  // Name of the input file to load the DSR graph from.
  std::string dsr_input_file_;
};

}  // namespace dsr_util

#endif  // DSR_UTIL__AGENT_NODE_HPP_
