cmake_minimum_required(VERSION 3.0.2)
project(mot_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(mot_perception_node src/mot_node.cpp)
target_link_libraries(mot_perception_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
