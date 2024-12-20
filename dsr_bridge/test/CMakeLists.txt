# Test for test_bridge_integration
ament_add_gtest(test_bridge_integration test_bridge_integration.cpp)
target_link_libraries(test_bridge_integration
  ${fastrtps_LIBRARIES}
  dsr_bridge
  ament_index_cpp::ament_index_cpp
)

# Test for test_bridge
ament_add_gtest(test_bridge test_bridge.cpp)
target_link_libraries(test_bridge
  ${fastrtps_LIBRARIES}
  dsr_bridge
  ament_index_cpp::ament_index_cpp
)
