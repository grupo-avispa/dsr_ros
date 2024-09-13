^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dsr_agents
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

X.X.X (XX-XX-2024)
------------------
* Improve DSR documentation.
* Better information and refactor in navAgent.
* Delete unseen persons in personAgent.
* Initial dsr_bridge.
* Added qRegisterMetaType.
* Added source to all agents.
* Refactor CMakelists.txt.
* Added new functions to agent node.
* Remove viewer as it's integrated in dsr_rqt_plugin.
* Remove QtExecutor as it's integrated in dsr_util.
* Move ros_to_dsr_types.hpp to dsr_util.
* Move agent_node.hpp to dsr_util and move agents.
* Skip the removal of person if they are interacting.
* Add the rooms to the DSR and the edges between them and the objects.
* Improve formating.
* Update license to Apache 2.0.
* Move DSR callback functions to agent_node.

0.7.0 (19-06-2024)
------------------
* Split the navigation agent into three agents: nav, dock and semantic.

0.6.2 (23-01-2024)
------------------
* Added try-catch in navAgent to handle UnknownGoalHandleError exceptions.

0.6.1 (19-01-2024)
------------------
* Update robot pose into the DSR when the navigation is updated in navAgent.
* Update personAgent 

0.6.0 (14-12-2023)
------------------
* Added function to get priority of nodes.
* Fix get_zones in navigation node.
* Battery agent will publish battery_percentage between 0 and 100.
* Cancel goal before sending a new one.
* Convert the power supply status from enum to string.
* Update robot.launch.py with navigation agent.
* Add direction and return node in add_node_with_edge function.
* Improve navigation agent: added 'abort', reorder 'wants_to' and 'cancel',
    modify edges in callbacks.
* Added person_agent.
* Update launch and params with person agent.
* Added campero launch and params files.

0.5.0 (07-11-2023)
------------------
* Move agents to 'agents' folder.
* Added new functions to delete and replace edges.
* Added new functions to add nodes and edges.
* Sort TF tree before publishing.
* Rename 'default' world to 'empty' world.
* Rename generic_agent to topic_agent.
* Added unfinished action_agent template.

0.4.0 (30-10-2023)
------------------
* Added QtExecutor to spin ROS events in a separate thread.
* Rename to dsr_agents.
* Added dock and semantic to nav agent.
* Added priority to nodes.
* Added TF agent to robot launch file.
* Update agents_id.

0.3.0 (05-07-2023)
------------------
* Added logging info in launch file.
* Added parameter for reading a world file.
* Added viewer.
* Generic and tf agents both inherit from agent node.
* Added agent_node class (.h and .cpp files).
* Added service to save dsr into a JSON file.
* Remove redundant agent_name.
* Added function to get random pos.
* Improve create node.
* Added RT update attributes to agent_node.
* Add sleeps to prevent unsynchronization with the DSR.
* Use frame_id from ROS message if parent_name is empty.

0.2.0 (25-04-2023)
------------------
* Prepare for humble release.

0.1.0 (03-03-2022)
------------------
* Initial release.
* Create README.md.
* Added generic agent class (.h and .cpp files).
* Added tf agent class (.h and .cpp files).
* Added launch files and config parameters.
* Contributors: Alberto Tudela
