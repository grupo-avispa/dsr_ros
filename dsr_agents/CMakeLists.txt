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
find_package(nav2_util REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(person_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# ##########
# # Build ##
# ##########

# Set DSR and FastRTPS
set(dsr_LIBRARIES dsr_api dsr_core dsr_gui)
set(fastrtps_LIBRARIES fastrtps fastcdr)

# Docking agent executable node
add_executable(docking_agent src/docking_agent.cpp)
target_include_directories(docking_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(docking_agent
  PUBLIC
  ${dsr_LIBRARIES}
  dsr_util::dsr_util
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${opennav_docking_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  tf2::tf2
)

# Navigation agent executable node
add_executable(nav_agent src/nav_agent.cpp)
target_include_directories(nav_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(nav_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  tf2::tf2
)
ament_target_dependencies(nav_agent PUBLIC nav2_util) # TODO(ajtudela): Fix this in jazzy

# Person agent executable node
add_executable(person_agent src/person_agent.cpp)
target_include_directories(person_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(person_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${person_msgs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)
ament_target_dependencies(person_agent PUBLIC nav2_util) # TODO(ajtudela): Fix this in jazzy

# Semantic navigation agent executable node
add_executable(semantic_nav_agent src/semantic_nav_agent.cpp)
target_include_directories(semantic_nav_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(semantic_nav_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${semantic_navigation_msgs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
)

# TF agent executable node
add_executable(tf_agent src/tf_agent.cpp)
target_include_directories(tf_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(tf_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${tf2_sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)
ament_target_dependencies(tf_agent PUBLIC nav2_util) # TODO(ajtudela): Fix this in jazzy

# Topic agent executable node
add_executable(topic_agent src/topic_agent.cpp)
target_include_directories(topic_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(topic_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${sensor_msgs_TARGETS}
)

# Whisper agent executable node
add_executable(whisper_agent src/whisper_agent.cpp)
target_include_directories(whisper_agent PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(whisper_agent
  PUBLIC
  ${dsr_LIBRARIES}
  rclcpp::rclcpp
  dsr_util::dsr_util
  PRIVATE
  ${fastrtps_LIBRARIES}
  ${std_msgs_TARGETS}
)

# ############
# # Install ##
# ############
install(TARGETS topic_agent person_agent whisper_agent tf_agent nav_agent docking_agent semantic_nav_agent
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
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
endif()

# Export old-style CMake variables
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  dsr_msgs
  dsr_util
  fastrtps
  fastcdr
  geometry_msgs
  nav2_msgs
  nav2_util
  opennav_docking_msgs
  person_msgs
  rclcpp
  rclcpp_action
  semantic_navigation_msgs
  sensor_msgs
  tf2
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  vision_msgs
)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_package()
