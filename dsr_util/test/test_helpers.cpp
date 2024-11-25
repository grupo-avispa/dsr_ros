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

#include "gtest/gtest.h"
#include "dsr_util/helpers.hpp"
#include "dsr/core/types/user_types.h"

TEST(DsrUtilTest, StringToAttribute) {
  // Test string
  auto att = dsr_util::helpers::string_to_attribute("test", 0);
  EXPECT_EQ(std::get<std::string>(att.value()), "test");

  // Test int
  att = dsr_util::helpers::string_to_attribute("5", 1);
  EXPECT_EQ(std::get<int>(att.value()), 5);

  // Test float
  att = dsr_util::helpers::string_to_attribute("5.0", 2);
  EXPECT_FLOAT_EQ(std::get<float>(att.value()), 5.0);

  // Test float nan
  att = dsr_util::helpers::string_to_attribute("nan", 2);
  EXPECT_TRUE(std::isnan(std::get<float>(att.value())));

  // Test float vector
  att = dsr_util::helpers::string_to_attribute("5.0,nan,7.0", 3);
  auto values = std::get<std::vector<float>>(att.value());
  EXPECT_FLOAT_EQ(values[0], 5.0);
  EXPECT_TRUE(std::isnan(values[1]));
  EXPECT_FLOAT_EQ(values[2], 7.0);

  // Test bool
  att = dsr_util::helpers::string_to_attribute("true", 4);
  EXPECT_TRUE(std::get<bool>(att.value()));

  // Test uint8_t vector
  att = dsr_util::helpers::string_to_attribute("5,6,7", 5);
  auto uint_values = std::get<std::vector<uint8_t>>(att.value());
  EXPECT_EQ(uint_values[0], 5);
  EXPECT_EQ(uint_values[1], 6);
  EXPECT_EQ(uint_values[2], 7);

  // Test uint32_t
  att = dsr_util::helpers::string_to_attribute("5", 6);
  EXPECT_EQ(std::get<uint32_t>(att.value()), 5);

  // Test uint64_t
  att = dsr_util::helpers::string_to_attribute("5", 7);
  EXPECT_EQ(std::get<uint64_t>(att.value()), 5);

  // Test double
  att = dsr_util::helpers::string_to_attribute("5.0", 8);
  EXPECT_DOUBLE_EQ(std::get<double>(att.value()), 5.0);

  // Test double nan
  att = dsr_util::helpers::string_to_attribute("nan", 8);
  EXPECT_TRUE(std::isnan(std::get<double>(att.value())));

  // Test unknown type
  att = dsr_util::helpers::string_to_attribute("test", 15);
}

TEST(DsrUtilTest, AttributeToString) {
  // Test string
  DSR::Attribute att;
  att.value("test");
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "test");

  // Test int
  att.value(5);
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5");

  // Test float
  att.value(static_cast<float>(5.0));
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5.000000");

  // Test float vector
  att.value(std::vector<float>{5.0, std::numeric_limits<float>::quiet_NaN(), 7.0});
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5.000000,nan,7.000000");

  // Test bool
  att.value(true);
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "true");

  // Test uint8_t vector
  att.value(std::vector<uint8_t>{5, 6, 7});
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5,6,7");

  // Test uint32_t
  att.value(static_cast<uint32_t>(5));
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5");

  // Test uint64_t
  att.value(static_cast<uint64_t>(5));
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5");

  // Test double
  att.value(5.0);
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "5.000000");

  // Test unknown type
  att.value(std::vector<uint64_t>{5, 6, 7});
  EXPECT_EQ(dsr_util::helpers::attribute_to_string(att), "");
}

TEST(DsrUtilTest, AttributesToString) {
  std::map<std::string, DSR::Attribute> atts;
  DSR::Attribute att;
  att.value(1.0);
  atts["pos_x"] = att;
  att.value(2.0);
  atts["pos_y"] = att;

  auto att_str = dsr_util::helpers::attributes_to_string(atts);
  EXPECT_EQ(att_str.size(), 6);
  EXPECT_EQ(att_str[0], "pos_x");
  EXPECT_EQ(att_str[1], "1.000000");
  EXPECT_EQ(att_str[2], "8");
  EXPECT_EQ(att_str[3], "pos_y");
  EXPECT_EQ(att_str[4], "2.000000");
  EXPECT_EQ(att_str[5], "8");
}

TEST(DsrUtilTest, AttributesToStringByNames) {
  auto node = DSR::Node::create<robot_node_type>("test");
  DSR::Attribute att;
  att.value(1.0);
  node.attrs().insert_or_assign("pos_x", att);
  att.value(2.0);
  node.attrs().insert_or_assign("pos_y", att);

  auto att_str = dsr_util::helpers::attributes_to_string_by_names(node, {"pos_x", "pos_y"});
  EXPECT_EQ(att_str.size(), 6);
  EXPECT_EQ(att_str[0], "pos_x");
  EXPECT_EQ(att_str[1], "1.000000");
  EXPECT_EQ(att_str[2], "8");
  EXPECT_EQ(att_str[3], "pos_y");
  EXPECT_EQ(att_str[4], "2.000000");
  EXPECT_EQ(att_str[5], "8");
}

TEST(DsrUtilTest, GetAttributeType) {
  DSR::Attribute att;
  att.value("test");
  EXPECT_EQ(dsr_util::helpers::get_type_from_attribute(att), "0");
}

TEST(DsrUtilTest, ModifyAttributesFromString) {
  auto node = DSR::Node::create<robot_node_type>("test");
  auto attrs_str = std::vector<std::string>{"pos_x", "1.0", "2", "pos_y", "2.0", "2"};
  dsr_util::helpers::modify_attributes_from_string(node, attrs_str);
  auto attrs = node.attrs();
  auto search = attrs.find("pos_x");
  EXPECT_TRUE(search != attrs.end());
  EXPECT_EQ(std::get<float>(search->second.value()), 1.0);
  search = attrs.find("pos_y");
  EXPECT_TRUE(search != attrs.end());
  EXPECT_EQ(std::get<float>(search->second.value()), 2.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
