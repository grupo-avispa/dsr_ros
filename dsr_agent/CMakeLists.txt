cmake_minimum_required(VERSION 3.8)
project(dsr_agent)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

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

###################
## GENERIC AGENT ##
###################

## Qt Moc
qt5_wrap_cpp(generic_moc include/${PROJECT_NAME}/generic_agent.hpp
  TARGET generic_agent
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

# Executable node
add_executable(generic_agent src/generic_agent.cpp src/generic_agent_node.cpp)
ament_target_dependencies(generic_agent
  rclcpp
  Eigen3
  Qt5
  sensor_msgs
)
target_link_libraries(generic_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(generic_agent PRIVATE ${generic_moc})

##############
## TF AGENT ##
##############

## Qt Moc 
qt5_wrap_cpp(tf_moc include/${PROJECT_NAME}/tf_agent.hpp
  TARGET tf_agent
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

# Executable node
add_executable(tf_agent src/tf_agent.cpp src/tf_agent_node.cpp)
ament_target_dependencies(tf_agent
  rclcpp
  Eigen3
  Qt5
  tf2_msgs
  tf2_sensor_msgs
)
target_link_libraries(tf_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(tf_agent PRIVATE ${tf_moc})

#############
## Install ##
#############
install(TARGETS generic_agent tf_agent
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
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
