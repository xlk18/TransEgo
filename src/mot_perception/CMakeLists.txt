cmake_minimum_required(VERSION 3.0.2)
project(mot_perception)

# 1. 优先获取环境变量中的 TensorRT 路径
set(TRT_ROOT $ENV{TENSORRT_DIR})
if(NOT TRT_ROOT)
    # 如果环境变量没刷出来，手动指定保底路径
    set(TRT_ROOT "/home/yyf/ThirdParty/TensorRT-8.6.1.6")
endif()
message(STATUS "TensorRT search root: ${TRT_ROOT}")

# 2. 基础依赖寻找
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  geometry_msgs
  tf
  tf2_ros
  tf2_eigen
  eigen_conversions
)

find_package(PCL REQUIRED)

# 3. 寻找 TensorRT 头文件 (增加你的 TAR 包路径作为 HINTS)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h 
  HINTS ${TRT_ROOT}/include
  NO_DEFAULT_PATH # 强制只在指定路径找，防止找到系统里残余的老版本
)

# 4. 寻找 TensorRT 库文件
find_library(NVINFER_LIB nvinfer HINTS ${TRT_ROOT}/lib NO_DEFAULT_PATH)
find_library(NVONNX_LIB nvonnxparser HINTS ${TRT_ROOT}/lib NO_DEFAULT_PATH)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions visualization_msgs geometry_msgs tf tf2_ros tf2_eigen eigen_conversions
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  /usr/local/cuda/include
)

# 5. 链接目录 (增加你的 TAR 包 lib 路径)
link_directories(
  ${TRT_ROOT}/lib
  /usr/local/cuda/lib64
)

add_executable(mot_node
  src/mot_node.cpp
  src/bounding_box.cpp
  src/kalman_filter.cpp
  src/hungarian_algorithm.cpp
  src/mot_node_core.cpp
)

# 6. 链接库 (使用找到的具体库变量，更安全)
target_link_libraries(mot_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${NVINFER_LIB}
  ${NVONNX_LIB}
  cudart
)