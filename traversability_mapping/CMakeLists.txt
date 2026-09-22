cmake_minimum_required(VERSION 3.8)
project(traversability_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(elevation_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree io)
find_package(OpenCV REQUIRED)

add_compile_options(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(ROS_DEPENDENCIES
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs pcl_conversions cv_bridge
  image_transport elevation_msgs
)

add_executable(traversability_filter src/traversability_filter.cpp)
add_executable(traversability_map src/traversability_map.cpp)
add_executable(traversability_prm src/traversability_prm.cpp src/library/kdtree.c)
add_executable(traversability_path src/traversability_path.cpp)

foreach(target traversability_filter traversability_map traversability_prm traversability_path)
  ament_target_dependencies(${target} ${ROS_DEPENDENCIES})
  target_link_libraries(${target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach()

install(TARGETS
  traversability_filter
  traversability_map
  traversability_prm
  traversability_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ launch/
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/traversability_mapping.yaml
  config/nav2_costmap.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
