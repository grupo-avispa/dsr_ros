// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "dsr_util/qt_executor.hpp"
#include "dsr_agents/nav_agent.hpp"

// LCOV_EXCL_START
int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<dsr_agents::NavigationAgent>();

  dsr_util::QtExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.start();

  auto res = app.exec();
  rclcpp::shutdown();
  return res;
}
// LCOV_EXCL_STOP
