cmake_minimum_required(VERSION 3.8)
project(ambula_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# Dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)           # <-- ADD (odom)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# Python (mixed C++ + Python package)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# -------------------------
# C++ Node: LQR Balance
# -------------------------
add_executable(lqr_balance_node script/lqr_control.cpp)

target_include_directories(lqr_balance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lqr_balance_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(lqr_balance_node Eigen3::Eigen)
target_compile_features(lqr_balance_node PUBLIC cxx_std_17)

install(TARGETS lqr_balance_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# C++ Node: PD Balance (Gravity) -> Wheel L/R in Twist.angular.x/y
# -------------------------
add_executable(pd_balance_wheel_twist_node script/pd_control.cpp)

target_include_directories(pd_balance_wheel_twist_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pd_balance_wheel_twist_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

target_compile_features(pd_balance_wheel_twist_node PUBLIC cxx_std_17)

install(TARGETS
  pd_balance_wheel_twist_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Python package + scripts
# -------------------------
# IMPORTANT:
# ament_python_install_package(<python_pkg_dir_name>)
# ต้องเป็น "ชื่อโฟลเดอร์แพ็กเกจ python" ที่อยู่ข้าง CMakeLists.txt ตาม doc :contentReference[oaicite:5]{index=5}
# ถ้าใน repo ของคุณมีโฟลเดอร์ชื่อ ambula_controller/ (เป็น python module) ให้ใช้ชื่อนั้น
ament_python_install_package(ambula_controller)

install(
  DIRECTORY
    src
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

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

# ถ้าโฟลเดอร์นี้เป็น python package แล้ว (มี __init__.py) ปกติไม่ต้อง install ซ้ำแบบ DIRECTORY
# แต่ถ้าคุณต้องการวางไฟล์เพิ่มไว้ใต้ lib ก็ปล่อยไว้ได้
install(DIRECTORY
  ambula_controller
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Testing
# -------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
