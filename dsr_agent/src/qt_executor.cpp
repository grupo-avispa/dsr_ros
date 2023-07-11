
/*
 * QT EXECUTOR
 *
 * Copyright (c) 2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of dsr_agents.
 * 
 * All rights reserved.
 *
 */

#include <QCoreApplication>

#include "dsr_agent/qt_executor.hpp"

QtExecutor::QtExecutor(const rclcpp::ExecutorOptions &args) : rclcpp::Executor(args){
	QObject::connect(this, &QtExecutor::on_new_work, this, &QtExecutor::spin_work, 
		Qt::ConnectionType::BlockingQueuedConnection);
}

QtExecutor::~QtExecutor(){
	thread_.join();
}

void QtExecutor::spin(){
}

void QtExecutor::start(){
	thread_ = std::thread([this](){
		while (rclcpp::ok(this->context_)){
			wait_for_work();
			if (rclcpp::ok(this->context_)) {
				// If we are shutting down, 
				// we must not call on_new_work because the QT Event loop is closed.
				on_new_work();
			}
		}
		QMetaObject::invokeMethod(QCoreApplication::instance(), "quit", 
			Qt::ConnectionType::QueuedConnection);
	});
}

void QtExecutor::spin_work(){
	spin_some();
}
