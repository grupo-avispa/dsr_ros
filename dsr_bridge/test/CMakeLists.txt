find_package(dsr_util REQUIRED)

# Test for test_bridge node
ament_add_gtest(test_bridge test_bridge.cpp)
target_link_libraries(test_bridge
  ${fastrtps_LIBRARIES}
  dsr_bridge
  ament_index_cpp::ament_index_cpp
)
