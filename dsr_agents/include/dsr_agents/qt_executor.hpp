
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

#ifndef DSR_AGENT__QT_EXECUTOR_HPP_
#define DSR_AGENT__QT_EXECUTOR_HPP_

// C++
#include <thread>

// QT
#include <QObject>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

/**
 * @brief Single-threaded executor implementation for mixing Qt and ROS events.
 * 
 */
class QtExecutor : public QObject, public rclcpp::Executor{
	Q_OBJECT
	public:
		/**
		 * @brief Default constructor. See the rclcpp::Executor documentation for more information.
		 * 
		 * @param args See the rclcpp::ExecutorOptions documentation for more information.
		 */
		QtExecutor(const rclcpp::ExecutorOptions &args = rclcpp::ExecutorOptions());

		/**
		 * @brief Default destructor. Destroy the Qt Executor object
		 * 
		 */
		virtual ~QtExecutor();

		/**
		 * @brief Single-threaded implementation of spin. This function will block until work 
		 * comes in, execute it, and keep blocking.
		 * 
		 */
		void spin();

		/**
		 * @brief Start the executor.
		 * 
		 */
		void start();

	private:
		/**
		 * @brief Call spin_some() from the QT thread.
		 * 
		 * @return Q_INVOKABLE 
		 */
		Q_INVOKABLE void spin_work();

		/**
		 * @brief Signal emitted when new work comes in.
		 * 
		 * @return Q_SIGNAL 
		 */
		Q_SIGNAL void on_new_work();
	
	private:
		std::thread thread_;
};

#endif  // DSR_AGENT__QT_EXECUTOR_HPP_