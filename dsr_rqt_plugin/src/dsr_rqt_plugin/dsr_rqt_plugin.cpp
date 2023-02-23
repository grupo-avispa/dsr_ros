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

DSRView::DSRView(): rqt_gui_cpp::Plugin(), widget_(0){
	//setObjectName("DSRView");
}

DSRView::~DSRView(){

}

/* Initialize the publishers and subscribers */
void DSRView::initPlugin(qt_gui_cpp::PluginContext& context){
	// TODO: Fix this
	G_ = std::make_shared<DSR::DSRGraph>(0, "DSRView", 255, "");
	widget_ = new QWidget();
	QMainWindow *window;
	graph_viewer_ = std::make_unique<DSR::DSRViewer>(window, G_, DSR::DSRViewer::view::tree, DSR::DSRViewer::view::graph);
	window->setWindowFlags(Qt::Widget | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint | Qt::Tool);
	context.addWidget(window);

	/*auto centralwidget = new QMainWindow(this);
	centralwidget->setWindowFlags(Qt::Widget);
	setCentralWidget(centralwidget);*/
}

void DSRView::shutdownPlugin(){
}

void DSRView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
}

void DSRView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
}
}

PLUGINLIB_EXPORT_CLASS(dsr_rqt_plugin::DSRView, rqt_gui_cpp::Plugin)