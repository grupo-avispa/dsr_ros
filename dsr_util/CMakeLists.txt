cmake_minimum_required(VERSION 3.8)
project(dsr_util)

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

# Set headers
set(HEADERS
  include/${PROJECT_NAME}/qt_executor.hpp
)

# Set sources
set(SOURCES
  src/qt_executor.cpp
)

set(dependencies
  rclcpp
  Eigen3
  Qt5
)

# # Qt Moc
qt5_wrap_cpp(qt_moc ${HEADERS}
  OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME} ${DSR_LIBRARIES} ${QT_LIBRARIES})
target_sources(${PROJECT_NAME} PRIVATE ${qt_moc})

# ############
# # Install ##
# ############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"

  # PATTERN ".svn" EXCLUDE
)

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dsr_util)
ament_export_dependencies(${dependencies})
ament_package()
