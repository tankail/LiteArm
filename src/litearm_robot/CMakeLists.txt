cmake_minimum_required(VERSION 3.5)
project(litearm_robot)

# Find ament_cmake for ROS2 integration
find_package(ament_cmake REQUIRED)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install path prefix" FORCE)
endif()

set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
set(PROJECT_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

add_subdirectory(third_part/lcm)
add_subdirectory(third_part/serial_cmake)

# find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found! Please install libserialport-dev.")
else()
  message(STATUS "Found libserialport library: ${SERIALPORT_LIBRARY}")
endif()

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

set(serial_SRCS
    src/serial_driver.cpp
    src/parse_robot_params.cpp
    src/hardware/motor.cpp
    src/hardware/canport.cpp
    src/hardware/canboard.cpp
    src/hardware/robot.cpp
    src/crc/crc8.cpp
    src/crc/crc16.cpp
    src/litearm/LiteArm.cpp
)

add_library(litearm_robot SHARED ${serial_SRCS})

target_include_directories(litearm_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hardware>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/litearm_robot>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/crc>
  $<INSTALL_INTERFACE:include/hardware>
  $<INSTALL_INTERFACE:include/litearm_robot>
  $<INSTALL_INTERFACE:include/motor_msg>
)

target_link_libraries(litearm_robot
    PUBLIC
        lcm
        yaml-cpp
        serial_cmake
        ${SERIALPORT_LIBRARY}   # libserialport
    PRIVATE
        rt
        pthread
)


add_executable(canboard_update example/canboard_update.cpp)
target_link_libraries(canboard_update litearm_robot)

add_executable(motor_feedack example/motor_feedack.cpp)
target_link_libraries(motor_feedack litearm_robot)

add_executable(motor_move_zero example/motor_move_zero.cpp)
target_link_libraries(motor_move_zero litearm_robot)

add_executable(motor_run example/motor_run.cpp)
target_link_libraries(motor_run litearm_robot)

add_executable(motor_msg_subscriber example/motor_msg_subscriber.cpp)
target_link_libraries(motor_msg_subscriber litearm_robot)

add_executable(parse_demo example/parse_demo.cpp)
target_link_libraries(parse_demo litearm_robot)

set_target_properties(litearm_robot PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${VERSION_MAJOR}
    PUBLIC_HEADER "include/hardware/robot.hpp;include/hardware/motor.hpp;include/hardware/canport.hpp;include/hardware/canboard.hpp;include/parse_robot_params.hpp;include/serial_struct.hpp;include/serial_driver.hpp;include/crc/crc8.hpp;include/crc/crc16.hpp;include/litearm_robot/LiteArm.hpp;msg/motor_msg/motor_msg.hpp"
)

install(TARGETS litearm_robot
    EXPORT litearm_robotTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(DIRECTORY include/ DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY msg/motor_msg/ DESTINATION include/motor_msg
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY robot_param/ DESTINATION share/litearm_robot/robot_param
    FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY urdf/ DESTINATION share/litearm_robot/urdf
    FILES_MATCHING PATTERN "*.urdf")

install(EXPORT litearm_robotTargets
    FILE litearm_robotTargets.cmake
    NAMESPACE litearm_robot::
    DESTINATION lib/cmake/litearm_robot
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/litearm_robotConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/litearm_robotConfig.cmake
    INSTALL_DESTINATION lib/cmake/litearm_robot
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/litearm_robotConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/litearm_robotConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/litearm_robotConfigVersion.cmake
    DESTINATION lib/cmake/litearm_robot
)
add_executable(motor_set_zero example/motor_set_zero.cpp)
target_link_libraries(motor_set_zero litearm_robot)

# LiteArm examples
add_executable(0_robot_get_state examples/0_robot_get_state.cpp)
target_link_libraries(0_robot_get_state litearm_robot ament_index_cpp::ament_index_cpp)

add_executable(0_robot_set_zero examples/0_robot_set_zero.cpp)
target_link_libraries(0_robot_set_zero litearm_robot ament_index_cpp::ament_index_cpp)

add_executable(1_PD_control examples/1_PD_control.cpp)
target_link_libraries(1_PD_control litearm_robot ament_index_cpp::ament_index_cpp)

add_executable(1_PosVel_control examples/1_PosVel_control.cpp)
target_link_libraries(1_PosVel_control litearm_robot ament_index_cpp::ament_index_cpp)

add_executable(test_motor_enable examples/test_motor_enable.cpp)
target_link_libraries(test_motor_enable litearm_robot ament_index_cpp::ament_index_cpp)

# Find Pinocchio for dynamics computation (optional)
find_package(pinocchio QUIET)

if(pinocchio_FOUND)
  message(STATUS "Found pinocchio, building 2_joint_impedance_control example")
  add_executable(2_joint_impedance_control examples/2_joint_impedance_control.cpp)
  target_link_libraries(2_joint_impedance_control litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building 3_cartesian_impedance_control example")
  add_executable(3_cartesian_impedance_control examples/3_cartesian_impedance_control.cpp)
  target_link_libraries(3_cartesian_impedance_control litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building pure_cartesian_impedance_control example")
  add_executable(pure_cartesian_impedance_control examples/pure_cartesian_impedance_control.cpp)
  target_link_libraries(pure_cartesian_impedance_control litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building cartesian_impedance_ab_motion example")
  add_executable(cartesian_impedance_ab_motion examples/cartesian_impedance_ab_motion.cpp)
  target_link_libraries(cartesian_impedance_ab_motion litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building right_arm_gravity_compensation example")
  add_executable(right_arm_gravity_compensation examples/right_arm_gravity_compensation.cpp)
  target_link_libraries(right_arm_gravity_compensation litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)

  message(STATUS "Building right_arm_gravity_calc example (diagnostic only)")
  add_executable(right_arm_gravity_calc examples/right_arm_gravity_calc.cpp)
  target_link_libraries(right_arm_gravity_calc litearm_robot pinocchio::pinocchio ament_index_cpp::ament_index_cpp)
else()
  message(STATUS "pinocchio not found, skipping impedance control examples")
endif()


# Install example executables so ros2 run can find them
install(TARGETS
    canboard_update
    motor_feedack
    motor_move_zero
    motor_run
    motor_msg_subscriber
    parse_demo
    motor_set_zero
    0_robot_get_state
    0_robot_set_zero
    1_PD_control
    1_PosVel_control
    test_motor_enable
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(pinocchio_FOUND)
  install(TARGETS
      2_joint_impedance_control
      3_cartesian_impedance_control
      pure_cartesian_impedance_control
      cartesian_impedance_ab_motion
      right_arm_gravity_compensation
      right_arm_gravity_calc
      RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Ament package setup for ROS2
ament_package()
