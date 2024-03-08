cmake_minimum_required(VERSION 3.8)
project(dsr_rqt_plugin)

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
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

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


set(dsr_rqt_plugin_SRCS
  src/dsr_rqt_plugin/dsr_rqt_plugin.cpp
)

set(dsr_rqt_plugin_HDRS
  include/dsr_rqt_plugin/dsr_rqt_plugin.hpp
)

qt5_wrap_cpp(dsr_rqt_plugin_MOCS ${dsr_rqt_plugin_HDRS})

# Library
add_library(${PROJECT_NAME} SHARED
  ${dsr_rqt_plugin_SRCS}
  ${dsr_rqt_plugin_MOCS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  Eigen3
  Qt5
  qt_gui_cpp
  rqt_gui_cpp
)
target_link_libraries(${PROJECT_NAME} ${DSR_LIBRARIES} ${QT_LIBRARIES})

############
## VIEWER ##
############

# Executable node
add_executable(viewer src/dsr_rqt_plugin/viewer.cpp)
ament_target_dependencies(viewer
  Eigen3
  Qt5
)
target_link_libraries(viewer ${DSR_LIBRARIES} ${QT_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_package()
