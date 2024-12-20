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

// ROS
#include <pluginlib/class_list_macros.hpp>
#include "dsr_rqt_plugin/dsr_rqt_plugin.hpp"

// LCOV_EXCL_START

namespace dsr_rqt_plugin
{

DSRView::DSRView()
: rqt_gui_cpp::Plugin(), widget_graph_(0), widget_tree_(0)
{
}

DSRView::~DSRView()
{
  G_.reset();
}

/* Initialize the publishers and subscribers */
void DSRView::initPlugin(qt_gui_cpp::PluginContext & context)
{
  // Start DSR graph
  G_ = std::make_shared<DSR::DSRGraph>(0, "DSRView", 255, "");
  // Set the DSR viewer
  QMainWindow * window = new QMainWindow();
  int current_opts = DSR::DSRViewer::view::tree | DSR::DSRViewer::view::graph |
    DSR::DSRViewer::view::scene | DSR::DSRViewer::view::osg;
  graph_viewer_ = std::make_unique<DSR::DSRViewer>(
    window, G_, current_opts, DSR::DSRViewer::view::graph);
  // FIXME: This is a hack to avoid the window to be shown
  window->setVisible(false);
  // Add the widgets to the user interface
  widget_graph_ = graph_viewer_->get_widget(DSR::DSRViewer::view::graph);
  widget_graph_->setWindowTitle("DSRGraph");
  context.addWidget(widget_graph_);
  widget_tree_ = graph_viewer_->get_widget(DSR::DSRViewer::view::tree);
  widget_tree_->setWindowTitle("DSRTree");
  context.addWidget(widget_tree_);
}

void DSRView::shutdownPlugin()
{
  G_.reset();
}

}  // namespace dsr_rqt_plugin

PLUGINLIB_EXPORT_CLASS(dsr_rqt_plugin::DSRView, rqt_gui_cpp::Plugin)

// LCOV_EXCL_STOP
