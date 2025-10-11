cmake_minimum_required(VERSION 3.8)
project(ambula_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_python_install_package(src)

install(
    DIRECTORY 
    src
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    src/leg_fk.py
    src/leg_ik.py
    src/joint_state_command_merge.py
    src/height_to_point.py
    src/s_curve_command.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    ambula_controller
    DESTINATION lib/${PROJECT_NAME}
)


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
