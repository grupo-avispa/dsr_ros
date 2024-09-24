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

#include <chrono>
#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "dsr_util/dsr_api_ext.hpp"

#ifndef TEST_DSR_SETUP_HPP_
#define TEST_DSR_SETUP_HPP_

class DSRGraphExtFixture : public dsr_util::DSRGraphExt
{
public:
  DSRGraphExtFixture(std::string name, uint32_t agent_identifier, std::string input_file)
  : dsr_util::DSRGraphExt(name, agent_identifier, input_file)
  {
  }
  ~DSRGraphExtFixture() = default;

  std::tuple<float, float> get_position_by_level_in_graph(const DSR::Node & parent)
  {
    return dsr_util::DSRGraphExt::get_position_by_level_in_graph(parent);
  }

  std::tuple<float, float> get_random_position_to_draw_in_graph()
  {
    return dsr_util::DSRGraphExt::get_random_position_to_draw_in_graph();
  }
};

class DsrUtilTest : public ::testing::Test
{
public:
  DsrUtilTest() = default;
  ~DsrUtilTest() = default;

  void SetUp() override
  {
    auto pkg = ament_index_cpp::get_package_share_directory("dsr_util");
    G_ = std::make_shared<DSRGraphExtFixture>("test", 2, pkg + "/test/test_dsr.json");
  }

  void TearDown() override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    G_.reset();
  }

protected:
  std::shared_ptr<DSRGraphExtFixture> G_;
};

#endif  // TEST_DSR_SETUP_HPP_
