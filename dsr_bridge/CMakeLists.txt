cmake_minimum_required(VERSION 3.5)
project(dsr_bridge)

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
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# ##########
# # Build ##
# ##########
# Set FastRTPS libraries
set(fastrtps_LIBRARIES fastrtps fastcdr)

# Add library
add_library(dsr_bridge SHARED src/dsr_bridge.cpp)
target_include_directories(dsr_bridge PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(dsr_bridge
  PUBLIC
  ${dsr_msgs_TARGETS}
  dsr_util::agent_node
  rclcpp::rclcpp
  PRIVATE
  ${fastrtps_LIBRARIES}
  rclcpp_components::component
)
ament_target_dependencies(dsr_bridge PUBLIC nav2_util) # TODO(ajtudela): Fix this in jazzy

# Add executable
add_executable(dsr_bridge_node src/dsr_bridge_node.cpp)
target_link_libraries(dsr_bridge_node
  PRIVATE
  dsr_bridge
  dsr_util::qt_executor
  ${fastrtps_LIBRARIES}
  rclcpp::rclcpp
)

# Register components
rclcpp_components_register_nodes(dsr_bridge "dsr_bridge::DSRBridge")

# ############
# # Install ##
# ############
install(TARGETS dsr_bridge
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS dsr_bridge_node
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
ament_export_libraries(dsr_bridge)
ament_export_dependencies(
  dsr_msgs
  dsr_util
  nav2_util
  rclcpp
  rclcpp_components
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_package()
