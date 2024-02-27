/*
 * DSR RQT PLUGIN ROS NODE
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agent.
 * 
 * All rights reserved.
 *
 */

// ROS
#include <pluginlib/class_list_macros.hpp>
#include "dsr_rqt_plugin/dsr_rqt_plugin.hpp"

namespace dsr_rqt_plugin{

DSRView::DSRView(): rqt_gui_cpp::Plugin(), widget_graph_(0), widget_tree_(0){
}

DSRView::~DSRView(){
}

/* Initialize the publishers and subscribers */
void DSRView::initPlugin(qt_gui_cpp::PluginContext& context){
	// Start DSR graph
	G_ = std::make_shared<DSR::DSRGraph>(0, "DSRView", 255, "");
	// Set the DSR viewer
	QMainWindow *window = new QMainWindow();
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

void DSRView::shutdownPlugin(){
	G_.reset();
}

void DSRView::saveSettings(qt_gui_cpp::Settings& plugin_settings, 
	qt_gui_cpp::Settings& instance_settings) const{
}

void DSRView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
	const qt_gui_cpp::Settings& instance_settings){
}
}

PLUGINLIB_EXPORT_CLASS(dsr_rqt_plugin::DSRView, rqt_gui_cpp::Plugin)