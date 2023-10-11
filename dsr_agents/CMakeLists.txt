cmake_minimum_required(VERSION 3.8)
project(dsr_agents)

# Default to C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(dsr_interfaces REQUIRED)
find_package(auto_docking_interfaces REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

# Set QT libraries and DSR libraries
set(QT_LIBRARIES Qt5::Widgets Qt5::OpenGL Qt5::Core)
set(DSR_LIBRARIES dsr_api dsr_core dsr_gui fastcdr fastrtps)

# Set headers
set(HEADERS
  include/${PROJECT_NAME}/agent_node.hpp
  include/${PROJECT_NAME}/qt_executor.hpp
)

# Set sources
set(SOURCES
  src/agent_node.cpp
  src/qt_executor.cpp
)

## Qt Moc
qt5_wrap_cpp(agent_moc ${HEADERS}
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

###################
## GENERIC AGENT ##
###################

# Executable node
add_executable(generic_agent ${SOURCES} src/generic_agent.cpp)
ament_target_dependencies(generic_agent
  rclcpp
  Eigen3
  Qt5
  sensor_msgs
  nav2_util
  dsr_interfaces
)
target_link_libraries(generic_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(generic_agent PRIVATE ${agent_moc})

######################
## NAVIGATION AGENT ##
######################

# Executable node
add_executable(nav_agent ${SOURCES} src/nav_agent.cpp)
ament_target_dependencies(nav_agent
  rclcpp
  Eigen3
  Qt5
  geometry_msgs
  nav2_util
  tf2
  dsr_interfaces
  auto_docking_interfaces
  semantic_navigation_msgs
)
target_link_libraries(nav_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(nav_agent PRIVATE ${agent_moc})

##############
## TF AGENT ##
##############

# Executable node
add_executable(tf_agent ${SOURCES} src/tf_agent.cpp)
ament_target_dependencies(tf_agent
  rclcpp
  Eigen3
  Qt5
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  dsr_interfaces
)
target_link_libraries(tf_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(tf_agent PRIVATE ${agent_moc})

############
## VIEWER ##
############

# Executable node
add_executable(viewer src/viewer.cpp)
ament_target_dependencies(viewer
  Eigen3
  Qt5
)
target_link_libraries(viewer ${DSR_LIBRARIES} ${QT_LIBRARIES})

#############
## Install ##
#############
install(TARGETS generic_agent tf_agent nav_agent viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch params worlds
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
