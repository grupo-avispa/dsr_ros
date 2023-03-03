/*
 * ROS ATTRIBUTES NAMES ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#ifndef DSR_AGENT__ROS_ATTR_NAME_HPP_

#include "dsr/core/types/type_checking/dsr_attr_name.h"

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
REGISTER_TYPE(battery_power_supply_status, int, false)
REGISTER_TYPE(battery_power_supply_health, int, false)
REGISTER_TYPE(battery_power_supply_technology, int, false)
REGISTER_TYPE(battery_present, bool, false)
REGISTER_TYPE(battery_cell_voltage, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_cell_temperature, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_location, std::string, false)
REGISTER_TYPE(battery_serial_number, std::string, false)


#endif  // DSR_AGENT__ROS_ATTR_NAME_HPP_