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
REGISTER_TYPE(battery_power_supply_status, std::string, false)
REGISTER_TYPE(battery_power_supply_health, int, false)
REGISTER_TYPE(battery_power_supply_technology, int, false)
REGISTER_TYPE(battery_present, bool, false)
REGISTER_TYPE(battery_cell_voltage, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_cell_temperature, std::reference_wrapper<const std::vector<float>>, false)
REGISTER_TYPE(battery_location, std::string, false)
REGISTER_TYPE(battery_serial_number, std::string, false)

// LCOV_EXCL_STOP

#endif  // DSR_UTIL__ROS_TO_DSR_TYPES_HPP_
