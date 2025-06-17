cmake_minimum_required(VERSION 3.15)
project(rgbd_vio)

# 编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_PARALLEL_LEVEL 4)
# add_compile_options(-Wall -g -O3 -fpermissive)

# 输出目录
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(camera_models REQUIRED)



add_library(${PROJECT_NAME} SHARED
    src/parameters.cpp
    src/marginalization_factor.cpp
    src/projection_quat.cpp
    src/rvio.cpp
    src/rvio_init.cpp
    src/rvio_plane.cpp
    src/rvio_dvio.cpp
    src/depth_factor.cpp
    src/plane_correct_pose.cpp
    src/visualization.cpp
    src/feature_manager.cpp
    src/initial/gmm_model.cpp
    src/initial/initial_alignment.cpp
    src/initial/initial_sfm.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_ex_rotation.cpp
    src/initial/translate_factor.cpp
    src/initial/solve_opt.cpp
    src/utility/utility.cpp
)
# 链接依赖库
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    nav_msgs
    tf2_ros
    pcl_ros
    pcl_conversions
    visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${ament_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /usr/include
    /usr/include/pcl-1.14
    src
    src/utility
    src/feature_tracker
    src/initial
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/rvio_syn_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    cv_bridge
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${ament_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /usr/include
    /usr/include/pcl-1.14
    src
    src/utility
    src/feature_tracker
    src/initial
)

add_executable(run_dvio_node src/run_dvio_node.cpp)
ament_target_dependencies(run_dvio_node
    rclcpp
    sensor_msgs
    cv_bridge
    nav_msgs
    tf2_ros
    pcl_ros
    pcl_conversions
    visualization_msgs
)
target_link_libraries(run_dvio_node
    ${PROJECT_NAME}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
)
target_include_directories(run_dvio_node PRIVATE
    ${ament_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /usr/include
    /usr/include/pcl-1.14
    src
    src/utility
    # src/feature_tracker
    src/initial
)

# 取消注释后启用
# add_executable(dvio_init_node dvio_init_node.cpp)
# target_link_libraries(dvio_init_node
#     ${PROJECT_NAME}
#     ${CERES_LIBRARIES}
#     ${OpenCV_LIBS}
#     rclcpp::rclcpp
#     sensor_msgs::sensor_msgs
#     cv_bridge::cv_bridge
# )

add_executable(feature_tracker_${PROJECT_NAME}
    src/feature_tracker/feature_tracker.cpp
    src/feature_tracker/feature_tracker_node.cpp
    src/parameters.cpp
)
ament_target_dependencies(feature_tracker_${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    message_filters
    pcl_conversions
    camera_models
)
target_link_libraries(feature_tracker_${PROJECT_NAME}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBS}
)
target_include_directories(feature_tracker_${PROJECT_NAME} PRIVATE
    ${ament_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
    /usr/include
    /usr/include/pcl-1.14
    src
    src/utility
    src/feature_tracker
    src/initial
    # camera_models/
)

# 取消注释后启用
# add_executable(show_feat_result feature_tracker/show_feat_result.cpp)
# ament_target_dependencies(
#     rclcpp
#     sensor_msgs
# )
# target_link_libraries(show_feat_result
#     ${OpenCV_LIBS}
# )

# 安装配置（可选）
install(TARGETS
    ${PROJECT_NAME}
    run_dvio_node
    ${PROJECT_NAME}_node
    # feature_tracker_${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（可选）
install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config result launch
  DESTINATION share/${PROJECT_NAME}
)


# 导出依赖
ament_export_dependencies(OpenCV Eigen3 Ceres camera_models)
ament_export_include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

ament_package()
