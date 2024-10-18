cmake_minimum_required(VERSION 3.5)
project(dsr_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
