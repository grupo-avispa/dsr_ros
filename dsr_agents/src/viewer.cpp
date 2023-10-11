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
#include <chrono>
#include <thread>

// DSR
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"

#include "dsr_agents/ros_to_dsr_types.hpp"

int main(int argc, char** argv){
	//QApplication app(argc, argv);
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G = std::make_shared<DSR::DSRGraph>(0, "dsr_viewer", 51, "");

	// Create viewer
	/*QMainWindow *window = new QMainWindow();
	int current_opts = DSR::DSRViewer::view::tree | DSR::DSRViewer::view::graph | 
		DSR::DSRViewer::view::scene | DSR::DSRViewer::view::osg;
	std::unique_ptr<DSR::DSRViewer> dsr_viewer = std::make_unique<DSR::DSRViewer>(
		window, G, current_opts, DSR::DSRViewer::view::graph);
	window->setWindowTitle(QString::fromStdString("dsr_viewer"));*/

	if (auto planner_node = G->get_node("planner"); !planner_node.has_value()){
		auto node = DSR::Node::create<planner_node_type>("planner");
		G->insert_node(node);
		std::cout<< "Node inserted" << std::endl;
	}
	if (auto move_node = G->get_node("move"); !move_node.has_value()){
		auto node = DSR::Node::create<move_node_type>("move");
		G->insert_node(node);
		std::cout<< "Node inserted" << std::endl;
	}

	if (auto move_node = G->get_node("move"); move_node.has_value()){
		G->add_or_modify_attrib_local<goal_x_att>(move_node.value(), static_cast<float>(1.0));
		G->add_or_modify_attrib_local<goal_y_att>(move_node.value(), static_cast<float>(-3.0));
		G->add_or_modify_attrib_local<goal_angle_att>(move_node.value(), static_cast<float>(0.0));
		G->update_node(move_node.value());
		std::cout<< "Node updated" << std::endl;
	}
	

	std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	auto start_edge = DSR::Edge::create<start_edge_type>(
		G->get_node("planner").value().id(), G->get_node("move").value().id());
	if (G->insert_or_assign_edge(start_edge)){
		std::cout<< "Edge inserted" << std::endl;
	}
	if (G->insert_or_assign_edge(start_edge)){
		std::cout<< "Edge inserted" << std::endl;
	}
	//app.setQuitOnLastWindowClosed( true );
	//return app.exec();
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	return 0;
}