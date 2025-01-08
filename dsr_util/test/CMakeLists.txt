find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(test_msgs REQUIRED)

# Test for action agent node
ament_add_gtest(test_action_agent test_action_agent.cpp)
target_link_libraries(test_action_agent
  ${fastrtps_LIBRARIES}
  action_agent
  ament_index_cpp::ament_index_cpp
  ${test_msgs_TARGETS}
)
ament_target_dependencies(test_action_agent nav2_util)


# Test for agent node
ament_add_gtest(test_agent_node test_agent_node.cpp)
target_link_libraries(test_agent_node
  agent_node
  ${fastrtps_LIBRARIES}
  ament_index_cpp::ament_index_cpp
  ${tf2_geometry_msgs_TARGETS}
)

# Test for dsr_api ext
ament_add_gtest(test_dsr_api_ext test_dsr_api_ext.cpp)
target_include_directories(test_dsr_api_ext PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(test_dsr_api_ext
  agent_node
  ${fastrtps_LIBRARIES}
  ${dsr_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

# Test for test_helpers
ament_add_gtest(test_helpers test_helpers.cpp)
target_include_directories(test_helpers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(test_helpers
  ${fastrtps_LIBRARIES}
  ${dsr_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

# Test for qt executor
ament_add_gtest(test_qt_executor test_qt_executor.cpp)
target_link_libraries(test_qt_executor
  qt_executor
  ${qt_LIBRARIES}
)