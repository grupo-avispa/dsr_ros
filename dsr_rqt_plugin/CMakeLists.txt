cmake_minimum_required(VERSION 3.5)
project(dsr_rqt_plugin)

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

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core OpenGL Widgets)
find_package(qt_gui_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)

# ##########
# # Build ##
# ##########
set(executable_name viewer)
set(library_name ${PROJECT_NAME})

# Set DSR, FastRTPS, and Qt libraries
set(dsr_LIBRARIES dsr_api dsr_core dsr_gui)
set(fastrtps_LIBRARIES fastrtps fastcdr)
set(qt_LIBRARIES Qt5::Core Qt5::OpenGL Qt5::Widgets)

# Library
add_library(${library_name} SHARED
  src/${PROJECT_NAME}/dsr_rqt_plugin.cpp
)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(${library_name}
  PUBLIC
  qt_gui_cpp::qt_gui_cpp
  rqt_gui_cpp::rqt_gui_cpp
  rclcpp::rclcpp
  PRIVATE
  ${dsr_LIBRARIES}
  Eigen3::Eigen
  ${fastrtps_LIBRARIES}
  ${qt_LIBRARIES}
)

# Executable node
add_executable(${executable_name} src/dsr_rqt_plugin/viewer.cpp)
target_include_directories(${executable_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(${executable_name}
  PRIVATE
  ${dsr_LIBRARIES}
  Eigen3::Eigen
  ${fastrtps_LIBRARIES}
  ${qt_LIBRARIES}
)

# ############
# # Install ##
# ############
install(TARGETS ${library_name} ${executable_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  eigen3_cmake_module
  qt_gui_cpp
  rclcpp
  rqt_gui_cpp
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()
