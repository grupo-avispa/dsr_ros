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

#include <QCoreApplication>

#include "dsr_util/qt_executor.hpp"

namespace dsr_util
{

QtExecutor::QtExecutor(const rclcpp::ExecutorOptions & args)
: rclcpp::Executor(args)
{
  QObject::connect(
    this, &QtExecutor::on_new_work, this, &QtExecutor::spin_work,
    Qt::ConnectionType::BlockingQueuedConnection);
}

QtExecutor::~QtExecutor()
{
  thread_.join();
}

void QtExecutor::spin()
{
}

void QtExecutor::start()
{
  thread_ = std::thread(
    [this]() {
      while (rclcpp::ok(this->context_)) {
        wait_for_work();
        if (rclcpp::ok(this->context_)) {
          // If we are shutting down,
          // we must not call on_new_work because the QT Event loop is closed.
          on_new_work();
        }
      }
      QMetaObject::invokeMethod(
        QCoreApplication::instance(), "quit", Qt::ConnectionType::QueuedConnection);
    });
}

void QtExecutor::spin_work()
{
  spin_some();
}

}  // namespace dsr_util
