/*
 * DSR VIEWER
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

// QT
#include "QApplication"
#include <QtWidgets>

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

int main(int argc, char** argv){
	QApplication app(argc, argv);
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G = std::make_shared<DSR::DSRGraph>(0, "dsr_viewer", 0, "");

	// Create viewer
	QMainWindow *window = new QMainWindow();
	int current_opts = DSR::DSRViewer::view::tree | DSR::DSRViewer::view::graph | 
		DSR::DSRViewer::view::scene | DSR::DSRViewer::view::osg;
	std::unique_ptr<DSR::DSRViewer> dsr_viewer = std::make_unique<DSR::DSRViewer>(
		window, G, current_opts, DSR::DSRViewer::view::graph);
	window->setWindowTitle(QString::fromStdString("dsr_viewer"));

	app.setQuitOnLastWindowClosed( true );
	return app.exec();
}