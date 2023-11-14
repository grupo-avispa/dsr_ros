/*
 * ROS TO DSR TYPES NAMES ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__ROS_TO_DSR_TYPES_HPP_

#include <dsr/core/types/type_checking/dsr_node_type.h>
#include <dsr/core/types/type_checking/dsr_edge_type.h>
#include <dsr/core/types/type_checking/dsr_attr_name.h>

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

REGISTER_NODE_TYPE(planner)
REGISTER_NODE_TYPE(navigation)
REGISTER_NODE_TYPE(search_goal)
REGISTER_NODE_TYPE(set_random_goal)
REGISTER_NODE_TYPE(move)
REGISTER_NODE_TYPE(docking)
REGISTER_NODE_TYPE(say)
REGISTER_NODE_TYPE(play)
REGISTER_NODE_TYPE(use_case)

REGISTER_EDGE_TYPE(abort)
REGISTER_EDGE_TYPE(detected)
REGISTER_EDGE_TYPE(not_detected)
REGISTER_EDGE_TYPE(stopped)
REGISTER_EDGE_TYPE(start)
REGISTER_EDGE_TYPE(is)
REGISTER_EDGE_TYPE(finished)
REGISTER_EDGE_TYPE(aborting)
REGISTER_EDGE_TYPE(wants_to)
REGISTER_EDGE_TYPE(is_performing)
REGISTER_EDGE_TYPE(navigating)
REGISTER_EDGE_TYPE(failed)

REGISTER_TYPE(priority, int, false)
REGISTER_TYPE(error_code, std::string, false)
REGISTER_TYPE(number, int, false)
REGISTER_TYPE(text, std::string, false)
REGISTER_TYPE(pose_x, float, false)
REGISTER_TYPE(pose_y, float, false)
REGISTER_TYPE(pose_angle, float, false)
REGISTER_TYPE(goal_x, float, false)
REGISTER_TYPE(goal_y, float, false)
REGISTER_TYPE(goal_z, float, false)
REGISTER_TYPE(goal_angle, float, false)
REGISTER_TYPE(zone, std::string, false)
REGISTER_TYPE(zones, std::string, false)
REGISTER_TYPE(use_case_id, std::string, false)

#endif  // DSR_AGENT__ROS_TO_DSR_TYPES_HPP_