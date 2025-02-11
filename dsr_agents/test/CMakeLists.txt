find_package(ament_index_cpp REQUIRED)
find_package(dsr_util REQUIRED)
find_package(nav2_util REQUIRED)

# Test for docking agent node
ament_add_gtest(test_docking_agent test_docking_agent.cpp)
target_link_libraries(test_docking_agent
  ${fastrtps_LIBRARIES}
  docking_agent
  ament_index_cpp::ament_index_cpp
  ${opennav_docking_msgs_TARGETS}
)
ament_target_dependencies(test_docking_agent nav2_util)

# Test for nav agent node
ament_add_gtest(test_nav_agent test_nav_agent.cpp)
target_link_libraries(test_nav_agent
  ${fastrtps_LIBRARIES}
  nav_agent
  ament_index_cpp::ament_index_cpp
  ${nav2_msgs_TARGETS}
)
ament_target_dependencies(test_nav_agent nav2_util)

# Test for tf agent node
ament_add_gtest(test_tf_agent test_tf_agent.cpp)
target_link_libraries(test_tf_agent
  ${fastrtps_LIBRARIES}
  tf_agent
  ament_index_cpp::ament_index_cpp
)

# Test for tf integration agent node
ament_add_gtest(test_tf_agent_integration test_tf_agent_integration.cpp)
target_link_libraries(test_tf_agent_integration
  ${fastrtps_LIBRARIES}
  tf_agent
  ament_index_cpp::ament_index_cpp
)

# Test for topic agent node
ament_add_gtest(test_topic_agent test_topic_agent.cpp)
target_link_libraries(test_topic_agent
  ${fastrtps_LIBRARIES}
  topic_agent
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
)

# Test for topic integration agent node
ament_add_gtest(test_topic_agent_integration test_topic_agent_integration.cpp)
target_link_libraries(test_topic_agent_integration
  ${fastrtps_LIBRARIES}
  topic_agent
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
)