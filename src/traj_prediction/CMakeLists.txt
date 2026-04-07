cmake_minimum_required(VERSION 3.0.2)
project(traj_prediction)

# 1. 设置 TensorRT 搜索根目录 (优先读环境变量)
set(TRT_ROOT $ENV{TENSORRT_DIR})
if(NOT TRT_ROOT)
    set(TRT_ROOT "/home/yyf/ThirdParty/TensorRT-8.6.1.6")
endif()
message(STATUS "Traj Prediction - TensorRT search root: ${TRT_ROOT}")

# 2. 基础依赖寻找
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy 
  std_msgs 
  geometry_msgs 
  visualization_msgs
  trajectory_msgs
)

# 3. 寻找 TensorRT 头文件和库文件
find_path(TENSORRT_INCLUDE_DIR NvInfer.h 
  HINTS ${TRT_ROOT}/include
  NO_DEFAULT_PATH
)
find_library(NVINFER_LIB nvinfer HINTS ${TRT_ROOT}/lib NO_DEFAULT_PATH)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TRT_ROOT}/lib NO_DEFAULT_PATH)
find_library(NVONNX_LIB nvonnxparser HINTS ${TRT_ROOT}/lib NO_DEFAULT_PATH)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs trajectory_msgs
)

# 4. 包含路径 (移除冗余的 aarch64 路径)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  /usr/local/cuda/include
)

# 5. 链接目录
link_directories(
  ${TRT_ROOT}/lib
  /usr/local/cuda/lib64
)

# 保留你原本的 Python 脚本设置
catkin_install_python(PROGRAMS 
  scripts/transformer_predict.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 6. 编译 C++ 推理节点
add_executable(trt_predictor_node src/trt_predictor_node.cpp)

# 7. 链接具体的库变量
target_link_libraries(trt_predictor_node
  ${catkin_LIBRARIES}
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  ${NVONNX_LIB}
  cudart
)