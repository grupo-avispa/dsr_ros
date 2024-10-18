find_package(dsr_util REQUIRED)

# Test for tf agent node
ament_add_gtest(test_tf_agent test_tf_agent.cpp)
target_link_libraries(test_tf_agent
  ${fastrtps_LIBRARIES}
  tf_agent
  ament_index_cpp::ament_index_cpp
)

# Test for tf integration agent node
ament_add_gtest(test_integration_tf_agent test_integration_tf_agent.cpp)
target_link_libraries(test_integration_tf_agent
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
ament_add_gtest(test_integration_topic_agent test_integration_topic_agent.cpp)
target_link_libraries(test_integration_topic_agent
  ${fastrtps_LIBRARIES}
  topic_agent
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
)