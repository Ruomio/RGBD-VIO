/*
	Oct. 22 2019, He Zhang, hzhang8@vcu.edu

	functions to display results

*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/image.h>
// #include <image_encodings.h>
#include <nav_msgs/msg/path.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include "tf/tf.h"
#include <tf2_ros/transform_broadcaster.h>
// #include "CameraPoseVisualization.h"
#include <cv_bridge/cv_bridge.hpp>
#include <eigen3/Eigen/Dense>
#include "nav_msgs/msg/odometry.hpp"
#include "rvio.h"

extern nav_msgs::msg::Path path;
extern std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_odometry;
extern std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pub_path, pub_pose;
extern std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_cloud, pub_map;
extern std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_key_poses;
extern std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_ref_pose, pub_cur_pose;
extern std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_key;
extern std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> pub_pose_graph;
extern std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_camera_pose;
extern int IMAGE_ROW, IMAGE_COL;

void registerPub(std::shared_ptr<rclcpp::Node> &n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);

void pubTrackImage(const cv::Mat &imgTrack, const double t);

// void printStatistics(const RVIO &RVIO, double t);

void pubOdometry(const RVIO &rvio, const std_msgs::msg::Header &header);

void pubCameraPose(const RVIO &estimator, const std_msgs::msg::Header &header);

void pubKeyPoses(const RVIO &rvio, const std_msgs::msg::Header &header);

// void pubCameraPose(const RVIO &RVIO, const std_msgs::Header &header);

void pubPointCloud(const RVIO &rvio, const std_msgs::msg::Header &header);

void pubTF(const RVIO &rvio, const std_msgs::msg::Header &header);

// void pubKeyframe(const RVIO &RVIO);

void pubFloorPoint(const RVIO &estimator, const std_msgs::msg::Header &header);

void pubNonFloorPoint(const RVIO &estimator, const std_msgs::msg::Header &header);

void pubPlaneCorrectedOdometry(tf2::Transform pose, const std_msgs::msg::Header &header);
