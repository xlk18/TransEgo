cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  tf2
  tf2_ros
  trajectory_msgs   
)
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${casadi_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
add_executable(advanced_mpc_node src/mpc_node_advanced_main.cpp src/quadrotor_nmpc_advanced.cpp)
target_link_libraries(advanced_mpc_node ${catkin_LIBRARIES} casadi)

add_executable(mpc_node src/mpc_node.cpp src/quadrotor_nmpc.cpp)
target_link_libraries(mpc_node ${catkin_LIBRARIES} casadi)
