cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  quadrotor_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mpc_controller_node src/mpc_node.cpp)
target_link_libraries(mpc_controller_node ${catkin_LIBRARIES})
