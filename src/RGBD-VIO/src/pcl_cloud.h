#pragma once
#include "pcl/impl/point_types.hpp"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 自定义点类型（如果ROS1的PointCloud包含多个通道）
struct CustomPoint {
    PCL_ADD_POINT4D;
    float intensity;             // 通道0
    float pu, pv;                // 通道1-2
    float vx, vy;                // 通道3-4
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, pu, pu)
    (float, pv, pv)
    (float, vx, vx)
    (float, vy, vy)
)
