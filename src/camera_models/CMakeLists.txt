cmake_minimum_required(VERSION 3.5)
project(camera_models)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# Include directories
# include_directories(
#     include
#     ${Boost_INCLUDE_DIRS}
#     ${CERES_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )

# Declare a library
add_library(camera_models SHARED
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc
)

target_include_directories(camera_models PUBLIC
    # include
    # ${PROJECT_INCLUDE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 仅构建时使用
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# Link libraries
ament_target_dependencies(camera_models PUBLIC
    rclcpp
    std_msgs
)
target_link_libraries(camera_models PUBLIC
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

# Declare an executable
# add_executable(Calibrations
#     src/intrinsic_calib.cc
#     src/chessboard/Chessboard.cc
#     src/calib/CameraCalibration.cc
#     src/camera_models/Camera.cc
#     src/camera_models/CameraFactory.cc
#     src/camera_models/CostFunctionFactory.cc
#     src/camera_models/PinholeCamera.cc
#     src/camera_models/PinholeFullCamera.cc
#     src/camera_models/CataCamera.cc
#     src/camera_models/EquidistantCamera.cc
#     src/camera_models/ScaramuzzaCamera.cc
#     src/sparse_graph/Transform.cc
#     src/gpl/gpl.cc
#     src/gpl/EigenQuaternionParameterization.cc
# )
#
# target_include_directories(Calibrations PRIVATE
#     include
#     ${Boost_INCLUDE_DIRS}
#     ${CERES_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )
# # Link libraries to executable
# ament_target_dependencies(Calibrations
#     rclcpp
#     std_msgs
# )
#
# target_link_libraries(Calibrations
#     camera_models
#     ${Boost_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${CERES_LIBRARIES}
# )

# Install targets
install(TARGETS
    camera_models
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS
#     Calibrations
#     ARCHIVE DESTINATION lib/${PROJECT_NAME}
#     LIBRARY DESTINATION lib/${PROJECT_NAME}
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Install include directories
install(DIRECTORY include/
    DESTINATION include/
)
# 安装导出目标
# install(EXPORT ${PROJECT_NAME}_export
#   FILE ${PROJECT_NAME}Targets.cmake
#   NAMESPACE ${PROJECT_NAME}::
#   DESTINATION share/${PROJECT_NAME}/cmake
# )

# 生成CMake配置文件
# include(CMakePackageConfigHelpers)
# configure_package_config_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
#   INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
# )
# Export dependencies
ament_export_dependencies(
    ament_cmake
    rclcpp
    std_msgs
    Boost
    OpenCV
    Ceres
)

ament_export_targets(
    ${PROJECT_NAME}_export
    HAS_LIBRARY_TARGET
)

ament_export_include_directories(
    install
)

ament_package()
