cmake_minimum_required(VERSION 3.0.2)
project(trans_ego_integration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/topic_rate_watchdog.py
  scripts/ablation_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
