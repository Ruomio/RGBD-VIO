cmake_minimum_required(VERSION 3.5)
project(rgbd_vio)

# 编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
add_compile_options(-Wall -g -O3 -fpermissive)

# 输出目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# camera_model 可能需要替换为对应 ROS2 包名，如果没有可用，需自行实现或调整

# include 头文件路径
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

# 子目录
add_subdirectory(rvio)

# 导出依赖
ament_export_dependencies(OpenCV Eigen3 Ceres)
ament_export_include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

ament_package()
