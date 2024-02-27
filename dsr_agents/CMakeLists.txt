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
find_package(vision_msgs REQUIRED)

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

# Set common dependencies
set(COMMON_DEPENDENCIES
  rclcpp
  Eigen3
  Qt5
  dsr_interfaces
  nav2_util
)

## Qt Moc
qt5_wrap_cpp(agent_moc ${HEADERS}
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

###################
## TOPIC AGENT ##
###################

# Executable node
add_executable(topic_agent ${SOURCES} src/agents/topic_agent.cpp)
ament_target_dependencies(topic_agent
  ${COMMON_DEPENDENCIES}
  sensor_msgs
)
target_link_libraries(topic_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(topic_agent PRIVATE ${agent_moc})

######################
## NAVIGATION AGENT ##
######################

# Executable node
add_executable(nav_agent ${SOURCES} src/agents/nav_agent.cpp)
ament_target_dependencies(nav_agent
  ${COMMON_DEPENDENCIES}
  geometry_msgs
  tf2
  auto_docking_interfaces
  semantic_navigation_msgs
)
target_link_libraries(nav_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(nav_agent PRIVATE ${agent_moc})

##################
## PERSON AGENT ##
##################

# Executable node
add_executable(person_agent ${SOURCES} src/agents/person_agent.cpp)
ament_target_dependencies(person_agent
  ${COMMON_DEPENDENCIES}
  geometry_msgs
  tf2
  vision_msgs
)
target_link_libraries(person_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(person_agent PRIVATE ${agent_moc})

##############
## TF AGENT ##
##############

# Executable node
add_executable(tf_agent ${SOURCES} src/agents/tf_agent.cpp)
ament_target_dependencies(tf_agent
  ${COMMON_DEPENDENCIES}
  geometry_msgs
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)
target_link_libraries(tf_agent ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(tf_agent PRIVATE ${agent_moc})

################
## DSR BRIDGE ##
################

# Executable node
add_executable(dsr_bridge ${SOURCES} src/dsr_bridge.cpp)
ament_target_dependencies(dsr_bridge
  ${COMMON_DEPENDENCIES}
  geometry_msgs
)
target_link_libraries(dsr_bridge ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(dsr_bridge PRIVATE ${agent_moc})

#############
## Install ##
#############
install(TARGETS topic_agent person_agent tf_agent nav_agent dsr_bridge
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
