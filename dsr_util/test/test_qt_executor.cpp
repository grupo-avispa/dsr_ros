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

#include <QCoreApplication>
#include <QtTest/QSignalSpy>
#include <thread>

#include "gtest/gtest.h"
#include "dsr_util/qt_executor.hpp"
#include "rclcpp/rclcpp.hpp"


class QtExecutorFixture : public dsr_util::QtExecutor
{
public:
  QtExecutorFixture()
  : dsr_util::QtExecutor()
  {
  }
  ~QtExecutorFixture() = default;

  void spin_work()
  {
    dsr_util::QtExecutor::spin_work();
  }
};

TEST(DSRQtExecutorTest, addNode)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  QtExecutorFixture executor;
  executor.add_node(node);

  // Sleep for a short time to verify executor.start() is going, and didn't throw.
  std::thread spinner([&]() {EXPECT_NO_THROW(executor.start());});
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Process Qt events to ensure signal delivery
  QCoreApplication::processEvents(QEventLoop::AllEvents, 5);

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
}

TEST(DSRQtExecutorTest, emptyExecutor)
{
  rclcpp::init(0, nullptr);
  QtExecutorFixture executor;

  std::thread spinner([&]() {EXPECT_NO_THROW(executor.spin());});

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  bool success = RUN_ALL_TESTS();
  return success;
}
