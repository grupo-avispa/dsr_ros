# Test for laser_segmentation integration
ament_add_gtest(test_dsr_api_ext test_dsr_api_ext.cpp)
target_link_libraries(test_dsr_api_ext
  ${FastRTPS_LIBRARIES}
  ${library_name}
  ament_index_cpp::ament_index_cpp
)