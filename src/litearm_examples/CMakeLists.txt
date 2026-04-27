cmake_minimum_required(VERSION 3.8)
project(litearm_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(litearm_joint_state_reader litearm_joint_state_reader.cpp)

ament_target_dependencies(litearm_joint_state_reader
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS litearm_joint_state_reader
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS test_single_joint.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
