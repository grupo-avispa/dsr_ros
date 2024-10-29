# dsr_bringup

## Overview

This package contains the launch files to bringup the DSR agents.

## Usage

To use this package, first modify the `default_params.yaml` file located in the `params` directory to set the configuration options for the generic_agent node.

Next, launch the package using the provided launch file `default.launch.py`. This file can be found in the launch directory of the package. To launch the package with default settings, use the following command:

	ros2 launch dsr_bringup default.launch.py

The package comes with an example configuration file called `robot_params.yaml` located in the `params` directory and an example launch file called `robot.launch.py` located in the launch directory. These files can be used as a starting point for creating custom configurations and launch files.

