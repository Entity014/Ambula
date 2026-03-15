cmake_minimum_required(VERSION 3.8)
project(ambula_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED) 
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# --- Include Directories ---
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# --- 1. opening_detector node (ตัวเดิมของคุณ) ---
add_executable(opening_detector
  src/opening_detection.cpp
)
ament_target_dependencies(opening_detector
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs 
  pcl_conversions pcl_ros tf2_ros tf2_eigen cv_bridge
)
target_link_libraries(opening_detector
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# --- 2. gap_detector node (ตัวใหม่ที่คุณเขียนมา) ---
add_executable(gap_detector
  src/gap_detector.cpp
)
ament_target_dependencies(gap_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(gap_detector
  ${PCL_LIBRARIES}
)

# --- Installation ---
install(TARGETS
  opening_detector
  gap_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/depth_perception.py
  script/person_detection.py
  script/hole_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()