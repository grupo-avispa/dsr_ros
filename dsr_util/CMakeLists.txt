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
find_package(dsr_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(rclcpp REQUIRED)
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

# Set headers
set(headers
  include/${PROJECT_NAME}/agent_node.hpp
  include/${PROJECT_NAME}/qt_executor.hpp
)

# Set sources
set(sources
  src/action_agent.cpp
  src/agent_node.cpp
  src/qt_executor.cpp
)

# # Qt Moc
qt5_wrap_cpp(qt_moc ${headers}
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

add_library(${library_name} SHARED ${sources})
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(${library_name}
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
target_sources(${library_name} PRIVATE ${qt_moc})

# ############
# # Install ##
# ############
install(TARGETS ${library_name}
  EXPORT ${library_name}
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
ament_export_libraries(dsr_util)
ament_export_dependencies(
  dsr_msgs
  eigen3_cmake_module
  geometry_msgs
  Qt5Core
  rclcpp
  rclcpp_lifecycle
  tf2_geometry_msgs
)

# Export modern CMake targets
ament_export_targets(${library_name})

ament_package()
