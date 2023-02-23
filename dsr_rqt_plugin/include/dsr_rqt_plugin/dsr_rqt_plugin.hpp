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

#ifndef DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_
#define DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_

// C++
#include <string>

// Qt
#include <QObject>

// ROS
#include "rqt_gui_cpp/plugin.h"

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

namespace dsr_rqt_plugin{

class DSRView : public rqt_gui_cpp::Plugin{
	Q_OBJECT
	public:
		DSRView();
		virtual ~DSRView();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

	protected:
		QWidget* widget_;

	private:
		// DSR
		std::shared_ptr<DSR::DSRGraph> G_;
		std::unique_ptr<DSR::DSRViewer> graph_viewer_;
};
}

#endif  // DSR_RQT_PLUGIN__DSR_RQT_PLUGIN_
