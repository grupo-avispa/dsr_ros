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

// QT
#include <QtWidgets>

// C++
#include <chrono>
#include <thread>

#include "QApplication"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

// LCOV_EXCL_START

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  // DSR graph
  std::shared_ptr<DSR::DSRGraph> G = std::make_shared<DSR::DSRGraph>("dsr_viewer", 801, "");

  // Create viewer
  QMainWindow * window = new QMainWindow();
  int current_opts = DSR::DSRViewer::view::tree | DSR::DSRViewer::view::graph |
    DSR::DSRViewer::view::scene | DSR::DSRViewer::view::osg;
  std::unique_ptr<DSR::DSRViewer> dsr_viewer = std::make_unique<DSR::DSRViewer>(
    window, G, current_opts, DSR::DSRViewer::view::graph);
  window->setWindowTitle(QString::fromStdString("dsr_viewer"));

  app.setQuitOnLastWindowClosed(true);
  return app.exec();
}

// LCOV_EXCL_STOP
