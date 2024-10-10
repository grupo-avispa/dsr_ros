find_package(tf2_geometry_msgs REQUIRED)

# Test for agent node
ament_add_gtest(test_agent_node test_agent_node.cpp)
target_link_libraries(test_agent_node
  ${fastrtps_LIBRARIES}
  ${library_name}
  ament_index_cpp::ament_index_cpp
  ${tf2_geometry_msgs_TARGETS}
)

# Test for dsr_api ext
ament_add_gtest(test_dsr_api_ext test_dsr_api_ext.cpp)
target_link_libraries(test_dsr_api_ext
  ${fastrtps_LIBRARIES}
  ${library_name}
  ament_index_cpp::ament_index_cpp
)

# Test for qt executor
ament_add_gtest(test_qt_executor test_qt_executor.cpp)
target_link_libraries(test_qt_executor
  ${library_name}
)