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

#ifndef DSR_UTIL__ROS_TO_DSR_TYPES_HPP_
#define DSR_UTIL__ROS_TO_DSR_TYPES_HPP_

#include <dsr/core/types/type_checking/dsr_node_type.h>
#include <dsr/core/types/type_checking/dsr_edge_type.h>
#include <dsr/core/types/type_checking/dsr_attr_name.h>

// C++
#include <string>
#include <vector>

// LCOV_EXCL_START

/**
 * sensor_msgs::msg::BatteryState
 */
REGISTER_TYPE(battery_voltage, float, false)
REGISTER_TYPE(battery_temperature, float, false)
REGISTER_TYPE(battery_current, float, false)
REGISTER_TYPE(battery_charge, float, false)
REGISTER_TYPE(battery_capacity, float, false)
REGISTER_TYPE(battery_design_capacity, float, false)
REGISTER_TYPE(battery_percentage, float, false)
REGISTER_TYPE(battery_power_supply_status, std::string, false)
REGISTER_TYPE(battery_power_supply_health, int, false)
REGISTER_TYPE(battery_power_supply_technology, int, false)
REGISTER_TYPE(battery_present, bool, false)
REGISTER_TYPE(battery_cell_voltage, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_cell_temperature, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_location, std::string, false)
REGISTER_TYPE(battery_serial_number, std::string, false)

/**
 * @brief CAMPERO
 *
 */

REGISTER_NODE_TYPE(brain)
REGISTER_NODE_TYPE(navigation)
REGISTER_NODE_TYPE(move)
REGISTER_NODE_TYPE(say)
REGISTER_NODE_TYPE(play)
REGISTER_NODE_TYPE(use_case)
REGISTER_NODE_TYPE(show)
REGISTER_NODE_TYPE(update_bbdd)
REGISTER_NODE_TYPE(tracking)
REGISTER_NODE_TYPE(bring_water)
REGISTER_NODE_TYPE(dock)
REGISTER_NODE_TYPE(get_random_goal)
REGISTER_NODE_TYPE(whisper)
REGISTER_NODE_TYPE(alarm)
REGISTER_NODE_TYPE(bed)
REGISTER_NODE_TYPE(listen)
REGISTER_NODE_TYPE(generate_response)
REGISTER_NODE_TYPE(generate_prompt)
REGISTER_NODE_TYPE(explanation)

REGISTER_EDGE_TYPE(stopped)
REGISTER_EDGE_TYPE(is)
REGISTER_EDGE_TYPE(is_performing)
REGISTER_EDGE_TYPE(is_with)
REGISTER_EDGE_TYPE(wants_to)
REGISTER_EDGE_TYPE(finished)
REGISTER_EDGE_TYPE(abort)
REGISTER_EDGE_TYPE(aborting)
REGISTER_EDGE_TYPE(cancel)
REGISTER_EDGE_TYPE(canceled)
REGISTER_EDGE_TYPE(failed)

// General types
REGISTER_TYPE(priority, int, false)
REGISTER_TYPE(result_code, std::string, false)
REGISTER_TYPE(number, int, false)
REGISTER_TYPE(source, std::string, false)
REGISTER_TYPE(timestamp, int, false)
REGISTER_TYPE(initstamp, int, false)
REGISTER_TYPE(posture, std::string, false)

// Navigation types
REGISTER_TYPE(pose_x, float, false)
REGISTER_TYPE(pose_y, float, false)
REGISTER_TYPE(pose_angle, float, false)
REGISTER_TYPE(goal_x, float, false)
REGISTER_TYPE(goal_y, float, false)
REGISTER_TYPE(goal_angle, float, false)
REGISTER_TYPE(dock_id, std::string, false)
REGISTER_TYPE(zone, std::string, false)
REGISTER_TYPE(zones, std::string, false)

// Play / say types
REGISTER_TYPE(text, std::string, false)
REGISTER_TYPE(sound, std::string, false)
REGISTER_TYPE(volume, float, false)

// RASA types
REGISTER_TYPE(port, int, false)
REGISTER_TYPE(intent, std::string, false)
REGISTER_TYPE(entity, std::string, false)

// Explanation types
REGISTER_TYPE(question, std::string, false)
REGISTER_TYPE(role, std::string, false)
REGISTER_TYPE(first_answer, std::string, false)
REGISTER_TYPE(answer, std::string, false)

// Show types
REGISTER_TYPE(interface, std::string, false)

// Use case types avialable: do_nothing, wandering, charging, menu, music, neuron_up,
// getme, reminder, announcer
REGISTER_TYPE(use_case_id, std::string, false)
REGISTER_TYPE(menu_choices, std::string, false)

// Person types
REGISTER_TYPE(identifier, std::string, false)
REGISTER_TYPE(safe_distance, float, false)
REGISTER_TYPE(comm_parameters, std::string, false)
REGISTER_TYPE(skills_parameters, std::string, false)
REGISTER_TYPE(menu, std::string, false)
REGISTER_TYPE(reminder, bool, false)
REGISTER_TYPE(activities, std::string, false)
REGISTER_TYPE(tracking_enable, bool, false)
REGISTER_TYPE(neuron, bool, false)
REGISTER_TYPE(track_id, std::string, false)

// BBDD types
REGISTER_TYPE(bbdd_agent, std::string, false)
REGISTER_TYPE(changes, std::string, false)

// LCOV_EXCL_STOP

#endif  // DSR_UTIL__ROS_TO_DSR_TYPES_HPP_
