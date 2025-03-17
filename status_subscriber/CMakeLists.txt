cmake_minimum_required(VERSION 3.8)
project(status_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(status_interfaces REQUIRED)
# find qt
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(qt_test src/qt_test.cpp)
add_executable(sys_status_sub src/sys_status_sub.cpp)

target_link_libraries(qt_test Qt5::Widgets)
target_link_libraries(sys_status_sub Qt5::Widgets)

ament_target_dependencies(
  sys_status_sub
  "status_interfaces"
  "rclcpp"
)

install(TARGETS
  sys_status_sub
  qt_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
