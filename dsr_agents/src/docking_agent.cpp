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


// DSR
#include "dsr_agents/docking_agent.hpp"

namespace dsr_agents
{

DockingAgent::DockingAgent(const rclcpp::NodeOptions & options)
: dsr_util::ActionAgent<opennav_docking_msgs::action::DockRobot>("docking_agent", options)
{
}

bool DockingAgent::get_goal_from_dsr(DSR::Node action_node)
{
  bool success = false;
  if (auto dock_id = G_->get_attrib_by_name<dock_id_att>(action_node); dock_id.has_value()) {
    goal_.dock_id = dock_id.value();
    success = true;
  }
  return success;
}
}  // namespace dsr_agents

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dsr_agents::DockingAgent)
