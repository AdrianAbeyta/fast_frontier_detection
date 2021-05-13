cmake_minimum_required(VERSION 3.0.2)
project(fast_frontier_detection)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry 
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fast_frontier_detection_node src/contour.cpp src/frontierDB.cpp src/ffd_main.cpp )
target_link_libraries(fast_frontier_detection_node ${catkin_LIBRARIES} )
add_dependencies(fast_frontier_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
