cmake_minimum_required(VERSION 3.8)
project(lab3)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cpe416_interfaces)

# create and install cpp executables
add_executable(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp std_msgs)

install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME})

# install python modules and executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  lab3/move_to_pose.py
  lab3/draw_square.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
