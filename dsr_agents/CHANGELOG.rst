^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dsr_agents
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
