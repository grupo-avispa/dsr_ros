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

#ifndef DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_HPP_
#define DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_HPP_

// Qt
#include <QObject>

// C++
#include <memory>
#include <string>

// ROS
#include "rqt_gui_cpp/plugin.h"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

namespace dsr_rqt_plugin
{

class DSRView : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  DSRView();
  virtual ~DSRView();
  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

protected:
  QWidget * widget_graph_;
  QWidget * widget_tree_;

private:
  // DSR
  std::shared_ptr<DSR::DSRGraph> G_;
  std::unique_ptr<DSR::DSRViewer> graph_viewer_;
};

}  // namespace dsr_rqt_plugin

#endif  // DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_HPP_
