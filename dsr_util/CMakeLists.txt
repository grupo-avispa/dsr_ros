cmake_minimum_required(VERSION 3.5)
project(dsr_util)

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
find_package(bond REQUIRED)
find_package(bondcpp REQUIRED)
find_package(dsr_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ##########
# # Build ##
# ##########
set(library_name ${PROJECT_NAME})

# Set DSR, FastRTPS, and Qt libraries
set(dsr_LIBRARIES dsr_api dsr_core)
set(fastrtps_LIBRARIES fastrtps fastcdr)
set(qt_LIBRARIES Qt5::Core)

# Qt Moc
qt5_wrap_cpp(qt_agent_moc include/${PROJECT_NAME}/agent_node.hpp
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)
qt5_wrap_cpp(qt_executor_moc include/${PROJECT_NAME}/qt_executor.hpp
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

# Add agent library
add_library(agent_node SHARED src/agent_node.cpp)
target_include_directories(agent_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(agent_node
  PUBLIC
  ${dsr_msgs_TARGETS}
  ${dsr_LIBRARIES}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${qt_LIBRARIES}
  PRIVATE
  Eigen3::Eigen
  ${fastrtps_LIBRARIES}
  ${tf2_geometry_msgs_TARGETS}
)
target_sources(agent_node PRIVATE ${qt_agent_moc})
ament_target_dependencies(agent_node PUBLIC bond bondcpp) # TODO(ajtudela): Fix this in jazzy

# Add action agent library
add_library(action_agent INTERFACE)
target_include_directories(action_agent INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(action_agent
  INTERFACE
  agent_node
  ${dsr_LIBRARIES}
  ${fastrtps_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# Add qt executor library
add_library(qt_executor SHARED src/qt_executor.cpp)
target_include_directories(qt_executor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(qt_executor
  PUBLIC
  rclcpp::rclcpp
  ${qt_LIBRARIES}
)
target_sources(qt_executor PRIVATE ${qt_executor_moc})


# ############
# # Install ##
# ############
install(TARGETS action_agent agent_node qt_executor
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES test/utils/test_dsr.json
  DESTINATION share/${PROJECT_NAME}/test
)

install(DIRECTORY test/utils/
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/utils
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
ament_export_libraries(agent_node qt_executor)
ament_export_dependencies(
  bondcpp
  dsr_msgs
  eigen3_cmake_module
  geometry_msgs
  Qt5Core
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_geometry_msgs
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_package()
