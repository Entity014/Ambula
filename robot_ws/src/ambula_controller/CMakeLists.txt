cmake_minimum_required(VERSION 3.8)
project(ambula_controller)

# --------------------------------------------------
# Compiler settings
# --------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# --------------------------------------------------
# C++ Executables
# --------------------------------------------------

# ====== convert_to_motor node ======
add_executable(convert_to_motor script/convert_to_motor.cpp)
ament_target_dependencies(convert_to_motor
  rclcpp
  sensor_msgs
  std_msgs
  Eigen3
)

# ====== height_adjust node ======
add_executable(height_adjust script/height_adjust.cpp)
ament_target_dependencies(height_adjust
  rclcpp
  sensor_msgs
  geometry_msgs
  Eigen3
)

# ====== inverse_kinematic node ======
add_executable(inverse_kinematic script/inverse_kinematic.cpp)
ament_target_dependencies(inverse_kinematic
  rclcpp
  sensor_msgs
  geometry_msgs
  Eigen3
)

# ====== joint_smoother node ======
add_executable(joint_smoother_cpp script/joint_smoother.cpp)
ament_target_dependencies(joint_smoother_cpp
  rclcpp
  sensor_msgs
  std_msgs
)

# Install C++ binaries
install(TARGETS
  convert_to_motor
  height_adjust
  inverse_kinematic
  joint_smoother_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Python package
# --------------------------------------------------

# ติดตั้ง python module folder
ament_python_install_package(ambula_controller)

# Install python executables
install(PROGRAMS
  src/joint_sim2real.py
  src/forward_kinematic.py
  src/inverse_kinematic.py
  src/leg_joint_cmd.py
  src/joint_smoother.py
  src/wheel_cmd.py
  src/joint_converter.py
  src/joint_state_demo.py
  src/joint_visualizer.py
  src/stand_feedforward.py
  src/balance_control.py
  src/position_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Launch / Config
# --------------------------------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Testing
# --------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
