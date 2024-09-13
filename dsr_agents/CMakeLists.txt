cmake_minimum_required(VERSION 3.8)
project(dsr_agents)

# Default to C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(dsr_msgs REQUIRED)
find_package(dsr_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(person_msgs REQUIRED)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
)

# Set QT libraries and DSR libraries
set(QT_LIBRARIES Qt5::Widgets Qt5::OpenGL Qt5::Core)
set(DSR_LIBRARIES dsr_api dsr_core dsr_gui fastcdr fastrtps)

# Set common dependencies
set(common_dependencies
  rclcpp
  Eigen3
  Qt5
  dsr_msgs
  dsr_util
  nav2_util
)

# ##################
# # TOPIC AGENT ##
# ##################

# Executable node
add_executable(topic_agent src/topic_agent.cpp)
ament_target_dependencies(topic_agent
  ${common_dependencies}
  sensor_msgs
)
target_link_libraries(topic_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# #####################
# # NAVIGATION AGENT ##
# #####################

# Executable node
add_executable(nav_agent src/nav_agent.cpp)
ament_target_dependencies(nav_agent
  ${common_dependencies}
  geometry_msgs
  tf2
  nav2_msgs
)
target_link_libraries(nav_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# #####################
# # DOCKING AGENT ##
# #####################

# Executable node
add_executable(docking_agent src/docking_agent.cpp)
ament_target_dependencies(docking_agent
  ${common_dependencies}
  geometry_msgs
  tf2
  opennav_docking_msgs
)
target_link_libraries(docking_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# ##############################
# # SEMANTIC NAVIGATION AGENT ##
# ##############################

# Executable node
add_executable(semantic_nav_agent src/semantic_nav_agent.cpp)
ament_target_dependencies(semantic_nav_agent
  ${common_dependencies}
  geometry_msgs
  tf2
  semantic_navigation_msgs
)
target_link_libraries(semantic_nav_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# #################
# # PERSON AGENT ##
# #################

# Executable node
add_executable(person_agent src/person_agent.cpp)
ament_target_dependencies(person_agent
  ${common_dependencies}
  geometry_msgs
  tf2
  vision_msgs
  person_msgs
)
target_link_libraries(person_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# #################
# # WHISPER AGENT ##
# #################

# Executable node
add_executable(whisper_agent src/whisper_agent.cpp)
ament_target_dependencies(whisper_agent
  ${common_dependencies}
  geometry_msgs
  tf2
  vision_msgs
  std_msgs
)
target_link_libraries(whisper_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# #############
# # TF AGENT ##
# #############

# Executable node
add_executable(tf_agent src/tf_agent.cpp)
ament_target_dependencies(tf_agent
  ${common_dependencies}
  geometry_msgs
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)
target_link_libraries(tf_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})

# ############
# # Install ##
# ############
install(TARGETS topic_agent person_agent whisper_agent tf_agent nav_agent docking_agent semantic_nav_agent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"

  # PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ##################################
# # ament specific configuration ##
# ##################################
ament_export_include_directories(include)
ament_export_dependencies(
  ${common_dependencies}
  geometry_msgs
  tf2
  sensor_msgs
  vision_msgs
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  auto_docking_interfaces
  semantic_navigation_msgs
)
ament_package()
