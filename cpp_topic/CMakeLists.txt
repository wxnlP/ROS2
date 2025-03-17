cmake_minimum_required(VERSION 3.8)
project(cpp_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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


# 添加可执行文件
add_executable(publisher_cpp src/publish_cpp.cpp)
add_executable(subscriber_cpp src/subscribe_cpp.cpp)
# 添加目标依赖
ament_target_dependencies(
  publisher_cpp
  "rclcpp"
  "std_msgs"
)
ament_target_dependencies(
  subscriber_cpp
  "rclcpp"
  "std_msgs"
)

install(TARGETS
  publisher_cpp
  subscriber_cpp
  DESTINATION lib/${PROJECT_NAME}
)
