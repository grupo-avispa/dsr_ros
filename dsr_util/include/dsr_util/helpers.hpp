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

#ifndef DSR_UTIL__HELPERS_HPP_
#define DSR_UTIL__HELPERS_HPP_

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

// ROS
#include "dsr/core/types/common_types.h"
#include "dsr/core/types/type_checking/dsr_edge_type.h"
#include "dsr/core/types/type_checking/dsr_node_type.h"

namespace dsr_util::helpers
{

/**
   * @brief Convert a string to a DSR::Attribute.
   *
   * @param att_value The string value of the attribute.
   * @param att_type The type of the attribute.
   * @return DSR::Attribute The DSR attribute created.
   */
DSR::Attribute string_to_attribute(const std::string & att_value, int att_type)
{
  DSR::Attribute new_att;
  switch (att_type) {
    case 0: {
        new_att.value(std::string(att_value));
        break;
      }
    case 1: {
        new_att.value(std::stoi(att_value));
        break;
      }
    case 2: {
        if (att_value == "nan") {
          new_att.value(std::numeric_limits<float>::quiet_NaN());
        } else {
          new_att.value(std::stof(att_value));
        }
        break;
      }
    case 3: {
        std::vector<std::string> values;
        boost::split(values, att_value, boost::is_any_of(","));
        std::vector<float> float_values;
        for (const auto & value : values) {
          if (value == "nan") {
            float_values.push_back(std::numeric_limits<float>::quiet_NaN());
          } else {
            float_values.push_back(std::stof(value));
          }
        }
        new_att.value(float_values);
        break;
      }
    case 4: {
        new_att.value(att_value == "true");
        break;
      }
    case 5: {
        std::vector<std::string> values;
        boost::split(values, att_value, boost::is_any_of(","));
        std::vector<uint8_t> uint_values;
        for (const auto & value : values) {
          uint_values.push_back(std::stoi(value));
        }
        new_att.value(uint_values);
        break;
      }
    case 6: {
        new_att.value(static_cast<std::uint32_t>(std::stoi(att_value)));
        break;
      }
    case 7: {
        new_att.value(std::stoull(att_value));
        break;
      }
    case 8: {
        if (att_value == "nan") {
          new_att.value(std::numeric_limits<double>::quiet_NaN());
        } else {
          new_att.value(std::stod(att_value));
        }
        break;
      }
    default:
      break;
  }
  return new_att;
}

/**
 * @brief Convert a DSR::Attribute to a string.
 *
 * @param att The DSR attribute.
 * @return std::string The string representation of the attribute.
 */
std::string attribute_to_string(const DSR::Attribute & att)
{
  std::locale::global(std::locale("C"));
  switch (att.value().index()) {
    case 0: {
        return std::get<std::string>(att.value());
      }
    case 1: {
        return std::to_string(std::get<int32_t>(att.value()));
      }
    case 2: {
        return std::to_string(std::get<float>(att.value()));
      }
    case 3: {
        std::string att_str;
        for (const auto & value : std::get<std::vector<float>>(att.value())) {
          att_str += std::to_string(value) + std::string(",");
        }
        att_str.pop_back();
        return att_str;
      }
    case 4: {
        return std::get<bool>(att.value()) ? "true" : "false";
      }
    case 5: {
        std::string att_str;
        for (const auto & value : std::get<std::vector<uint8_t>>(att.value())) {
          att_str += std::to_string(value) + std::string(",");
        }
        att_str.pop_back();
        return att_str;
      }
    case 6: {
        return std::to_string(std::get<uint32_t>(att.value()));
      }
    case 7: {
        return std::to_string(std::get<uint64_t>(att.value()));
      }
    case 8: {
        return std::to_string(std::get<double>(att.value()));
      }
    default:
      return "";
  }
}

/**
   * @brief Get the type from attribute object
   *
   * @param att The DSR attribute.
   * @return std::string The string representation of the attribute.
   */
std::string get_type_from_attribute(const DSR::Attribute & att)
{
  return std::to_string(att.value().index());
}

/**
 * @brief Convert a map of attributes to a vector of strings.
 *
 * @param atts The map of attributes.
 * @return std::vector<std::string> The vector of strings.
 */
std::vector<std::string> attributes_to_string(const std::map<std::string, DSR::Attribute> & atts)
{
  std::vector<std::string> att_vector_str;
  for (const auto & [att_name, att_value] : atts) {
    att_vector_str.push_back(att_name);
    att_vector_str.push_back(dsr_util::helpers::attribute_to_string(att_value));
    att_vector_str.push_back(dsr_util::helpers::get_type_from_attribute(att_value));
  }
  return att_vector_str;
}

/**
 * @brief Get the attributes from a DSR element in a vector of strings.
 *
 * @param elem The DSR element (node or edge) to modify.
 * @param atts_names The names of the attributes to convert.
 * @return std::vector<std::string> The vector of strings.
 */
template<typename TYPE>
std::vector<std::string> attributes_to_string_by_names(
  TYPE & elem, const std::vector<std::string> & atts_names)
{
  std::vector<std::string> att_vector_str;
  for (const auto & att_name : atts_names) {
    auto search = elem.attrs().find(att_name);
    if (search != elem.attrs().end()) {
      att_vector_str.push_back(att_name);
      att_vector_str.push_back(dsr_util::helpers::attribute_to_string(search->second));
      att_vector_str.push_back(dsr_util::helpers::get_type_from_attribute(search->second));
    }
  }
  return att_vector_str;
}

/**
   * @brief Modify the attributes of a DSR element with the given attributes in a vector of strings.
   *
   * @tparam TYPE Type of the DSR element.
   * @param elem The DSR element (node or edge) to modify.
   * @param att_str The attributes to modify in format (name, value, type).
   */
template<typename TYPE>
void modify_attributes_from_string(TYPE & elem, const std::vector<std::string> & att_str)
{
  for (unsigned int i = 0; i < att_str.size(); i += 3) {
    std::string att_name = att_str[i];
    std::string att_value = att_str[i + 1];
    std::string att_type = att_str[i + 2];

    // Add the attribute to the element
    DSR::Attribute new_att =
      dsr_util::helpers::string_to_attribute(att_value, std::stoi(att_type));
    elem.attrs().insert_or_assign(att_name, new_att);
  }
}

}  // namespace dsr_util::helpers

#endif  // DSR_UTIL__HELPERS_HPP_
