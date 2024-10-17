find_package(tf2_geometry_msgs REQUIRED)

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
  ament_index_cpp::ament_index_cpp
)

# Test for qt executor
ament_add_gtest(test_qt_executor test_qt_executor.cpp)
target_link_libraries(test_qt_executor
  qt_executor
  ${qt_LIBRARIES}
)