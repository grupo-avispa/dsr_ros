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

#ifndef DSR_UTIL__QT_EXECUTOR_HPP_
#define DSR_UTIL__QT_EXECUTOR_HPP_

// QT
#include <QObject>

// C++
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace dsr_util
{

/**
 * @class dsr_util::QtExecutor
 * @brief Single-threaded executor implementation for mixing Qt and ROS events.
 */
class QtExecutor : public QObject, public rclcpp::Executor
{
  Q_OBJECT

public:
  RCLCPP_SMART_PTR_DEFINITIONS(QtExecutor)

  /**
   * @brief Default constructor. See the rclcpp::Executor documentation for more information.
   *
   * @param args See the rclcpp::ExecutorOptions documentation for more information.
   */
  RCLCPP_PUBLIC
  explicit QtExecutor(const rclcpp::ExecutorOptions & args = rclcpp::ExecutorOptions());

  /**
   * @brief Default destructor.
   */
  RCLCPP_PUBLIC

  /**
   * @brief Destroy the Qt Executor object.
   */
  virtual ~QtExecutor();

  /**
   * @brief Single-threaded implementation of spin. This function will block until work
   * comes in, execute it, and keep blocking.
   */
  RCLCPP_PUBLIC
  void spin();

  /**
   * @brief Start the executor.
   */
  RCLCPP_PUBLIC
  void start();

private:
  /**
   * @brief Call spin_some() from the QT thread.
   *
   * @return Q_INVOKABLE
   */
  Q_INVOKABLE void spin_work();

  /**
   * @brief Signal emitted when new work comes in.
   *
   * @return Q_SIGNAL
   */
  Q_SIGNAL void on_new_work();

private:
  RCLCPP_DISABLE_COPY(QtExecutor)

  std::thread thread_;
};

}  // namespace dsr_util

#endif  // DSR_UTIL__QT_EXECUTOR_HPP_
