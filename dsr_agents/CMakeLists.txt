cmake_minimum_required(VERSION 3.5)
project(dsr_agents)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

option(COVERAGE_ENABLED "Enable code coverage" FALSE)

if(COVERAGE_ENABLED)
  add_compile_options(--coverage)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

# Defaults for Microsoft C++ compiler
if(MSVC)
  # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(dsr_msgs REQUIRED)
find_package(dsr_util REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ##########
# # Build ##
# ##########

# Set FastRTPS libraries
set(fastrtps_LIBRARIES fastrtps fastcdr)

# Docking agent library node
add_library(docking_agent SHARED src/docking_agent.cpp)
target_include_directories(docking_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(docking_agent
  PUBLIC
  dsr_util::action_agent
  ${opennav_docking_msgs_TARGETS}
  PRIVATE
  rclcpp_components::component
)

# Add docking agent executable
add_executable(docking_agent_node src/docking_agent_node.cpp)
target_link_libraries(docking_agent_node
  PRIVATE
  docking_agent
  dsr_util::qt_executor
  ${fastrtps_LIBRARIES}
  rclcpp::rclcpp
)

# Navigation agent library node
add_library(nav_agent SHARED src/nav_agent.cpp)
target_include_directories(nav_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav_agent
  PUBLIC
  dsr_util::action_agent
  ${nav2_msgs_TARGETS}
  PRIVATE
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${tf2_geometry_msgs_TARGETS}
)

# Add navigation agent executable
add_executable(nav_agent_node src/nav_agent_node.cpp)
target_link_libraries(nav_agent_node
  PRIVATE
  dsr_util::qt_executor
  ${fastrtps_LIBRARIES}
  nav_agent
  rclcpp::rclcpp
)

# TF agent library node
add_library(tf_agent SHARED src/tf_agent.cpp)
target_include_directories(tf_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(tf_agent
  PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  dsr_util::node_agent
  ${tf2_msgs_TARGETS}
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  rclcpp_components::component
  ${tf2_sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# Add TF agent executable
add_executable(tf_agent_node src/tf_agent_node.cpp)
target_link_libraries(tf_agent_node
  PRIVATE
  dsr_util::qt_executor
  ${fastrtps_LIBRARIES}
  rclcpp::rclcpp
  tf_agent
)

# Topic agent library node
add_library(topic_agent SHARED src/topic_agent.cpp)
target_include_directories(topic_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(topic_agent
  PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  dsr_util::node_agent
  PRIVATE
  ${fastrtps_LIBRARIES}
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# Add topic agent executable
add_executable(topic_agent_node src/topic_agent_node.cpp)
target_link_libraries(topic_agent_node
  PRIVATE
  dsr_util::qt_executor
  ${fastrtps_LIBRARIES}
  rclcpp::rclcpp
  topic_agent
)

# Register components
rclcpp_components_register_nodes(docking_agent "dsr_agents::DockingAgent")
rclcpp_components_register_nodes(nav_agent "dsr_agents::NavAgent")
rclcpp_components_register_nodes(tf_agent "dsr_agents::TFAgent")
rclcpp_components_register_nodes(topic_agent "dsr_agents::TopicAgent")

# ############
# # Install ##
# ############
install(TARGETS docking_agent nav_agent tf_agent topic_agent
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS docking_agent_node nav_agent_node tf_agent_node topic_agent_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

# Export old-style CMake variables
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(docking_agent nav_agent tf_agent topic_agent)
ament_export_dependencies(
  dsr_msgs
  dsr_util
  fastrtps
  fastcdr
  geometry_msgs
  nav2_msgs
  opennav_docking_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_package()
