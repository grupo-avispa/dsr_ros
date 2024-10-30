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

#ifndef DSR_UTIL__SERVICE_AGENT_HPP_
#define DSR_UTIL__SERVICE_AGENT_HPP_

// C++
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"

// DSR
#include "dsr_util/agent_node.hpp"

namespace wasp_dsr_agents
{

/**
 * @class dsr_agents::ServiceAgent
 * @brief Wrapper to call services from the DSR graph. Similar to ActionAgent but for services.
 * This class is used to call services from the DSR graph and update the graph with the results.
 * This class has callbacks that listen a 'wants_to' edge between the robot and the service node.
 * When the robot wants to start the service, the agent calls the ROS 2 service server and waits
 * for the result. The agent also listens to 'abort' and 'cancel' edges to cancel the service.
 *
 * @tparam ServiceT The type of the ROS 2 service to call.
 */
template<class ServiceT>
class ServiceAgent : public dsr_util::AgentNode
{
public:
  /**
   * @brief Construct a new Service Agent object.
   *
   * @param ros_node_name The name of the ROS node.
   * @param ros_service_name The name of the ROS service.
   * @param options The options for the ROS node.
   */
  ServiceAgent(
    std::string ros_node_name, std::string ros_service_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : dsr_util::AgentNode(ros_node_name, options), ros_service_name_(ros_service_name)
  {
  }

  /**
   * @brief Destroy the Action Agent object.
   */
  ~ActionAgent() = default;

  /**
   * @brief Configure the node
   *
   * @param state State of the node
   * @return CallbackReturn
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    // DSR parameters
    // If the action name is not set, use the ROS node name
    declare_parameter_if_not_declared(
      this, "dsr_action_name", rclcpp::ParameterValue(ros_service_name_),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The name of the action in the DSR graph"));
    this->get_parameter("dsr_action_name", dsr_action_name_);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter dsr_action_name is set to: [%s]", dsr_action_name_.c_str());

    int wait_for_service_timeout_s;
    declare_parameter_if_not_declared(
      this, "wait_for_service_timeout", rclcpp::ParameterValue(1000),
      rcl_interfaces::msg::ParameterDescriptor()
      .set__description("The timeout value for waiting for a service to response"));
    this->get_parameter("wait_for_service_timeout", wait_for_service_timeout_s);
    RCLCPP_INFO(
      this->get_logger(),
      "The parameter wait_for_service_timeout is set to: [%i]", wait_for_service_timeout_s);
    wait_for_service_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<int>(wait_for_service_timeout_s));

    create_service_client(ros_service_name_);

    return AgentNode::on_configure(state);
  }

protected:
  /**
   * @brief Function to perform some user-defined operation when the goal is received from the DSR.
   * Usually, this function should fill the goal message with the data from the DSR node.
   */
  virtual void get_goal_from_dsr(DSR::Node service_node) = 0;

  /**
   * @brief Function to perform some user-defined operation when the response is received
   * from the service.
   * Usually, this function should update the DSR graph with the response data.
   *
   * @param response
   */
  virtual void get_response(ServiceT::Response response) = 0;

  /**
   * @brief Create instance of an service client
   * @param ros_service_name Service name to create client for
   */
  void create_service_client(const std::string & ros_service_name)
  {
    // Now that we have the ROS node to use, create the service client for this service
    service_client_ = this->create_client<ServiceT>(ros_service_name);

    // Make sure the service is actually there before continuing
    RCLCPP_DEBUG(this->get_logger(), "Waiting for \"%s\" service", ros_service_name.c_str());
    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        this->get_logger(), "\"%s\" service not available after waiting for %.2fs",
        ros_service_name.c_str(),
        wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
              std::string("Service ") + ros_service_name + std::string(" not available"));
    }
  }

  /**
 * @brief Function to send new request to the ROS 2 service.
 */
  void send_new_request()
  {
    auto future_result = goals_generator_client_->async_send_request(
      request_,
      [this](rclcpp::Client<ServiceT>::SharedFuture future) {
        get_response(future.get());
      });
  }

  /**
   * @brief Callback executed when an edge is updated in the DSR graph.
   *
   * @param from The id of the parent node.
   * @param to The id of the child node.
   * @param type The type of the edge.
   */
  void edge_updated(std::uint64_t from, std::uint64_t to, const std::string & type) override
  {
    // Check if the robot wants to abort or cancel the action: robot ---(abort)--> action
    if (type == "abort" || type == "cancel") {
      auto robot_node = G_->get_node(from);
      auto action_node = G_->get_node(to);
      if (robot_node.has_value() && robot_node.value().name() == source_ &&
        action_node.has_value() && action_node.value().type() == dsr_action_name_)
      {
        RCLCPP_INFO(
          this->get_logger(), "Aborting / canceling the action '%s'", dsr_action_name_.c_str());
        // Delete the action node from the DSR graph
        if (G_->delete_node(action_node.value())) {
          RCLCPP_INFO(
            this->get_logger(), "Action '%s' has been canceled", dsr_action_name_.c_str());
        }
      }
      // Check if the robot wants to start the action: robot ---(wants_to)--> action
    } else if (type == "wants_to") {
      auto robot_node = G_->get_node(from);
      auto action_node = G_->get_node(to);
      if (robot_node.has_value() && robot_node.value().name() == source_ &&
        action_node.has_value() && action_node.value().type() == dsr_action_name_)
      {
        get_goal_from_dsr(action_node.value());
        send_new_request();
        RCLCPP_INFO(this->get_logger(), "Starting the service '%s'", dsr_action_name_.c_str());
      }
    }
  }

  // DSR action name
  std::string dsr_action_name_;

  // ROS service name
  std::string ros_service_name_;

  // The client to call the ROS 2 service
  rclcpp::Client<ServiceT>::SharedPtr service_client_;

  // The ROS 2 service request
  std::make_shared<ServiceT::Request> request_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;
}

}  // namespace wasp_dsr_agents

#endif  // DSR_UTIL__SERVICE_AGENT_HPP_
