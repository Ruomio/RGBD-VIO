/*
    Dec. 11, 2019, He Zhang, hzhang8@vcu.edu

    synchronize msg similar to VINS-Mono

*/

#include <rclcpp/logger.hpp>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "rvio.h"
#include "parameters.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization.h"
#include "pcl_cloud.h"


#define R2D(r) ((r)*180./M_PI)

RVIO rvio;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> feature_buf;
queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

std::mutex m_dpt_buf; // depth
queue<sensor_msgs::msg::Image::ConstSharedPtr> dpt_img_buf;

bool PUB_DEPTH_IMAGE = false;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_depth_img;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

// double nG = 1.0; //  -9.8;

bool DEPTH_INTERPOLATE = false;

sensor_msgs::msg::Image::ConstSharedPtr getDptImage(double timestamp)
{
    // RCLCPP_WARN("rvio_syn_node.cpp: try to find out dpt img at %lf", timestamp);
    while(true){
        if(dpt_img_buf.empty())
            return NULL;
        sensor_msgs::msg::Image::ConstSharedPtr dpt_img = dpt_img_buf.front();
        double current_time = rclcpp::Time(dpt_img->header.stamp).seconds();

        if(fabs(current_time - timestamp) < 1e-3){
            // RCLCPP_DEBUG("rvio_syn_node.cpp: found syn depth image at timestamp: %lf", current_time);
            return dpt_img;
        }else if(current_time < timestamp){
            dpt_img_buf.pop();
            // RCLCPP_INFO("rvio_syn_node.cpp: remove older depth at timestamp: %lf", current_time);
            continue;
        }else {
            RCLCPP_ERROR(rclcpp::get_logger("rvio_syn"), "rvio_syn_node.cpp: what? cannot find syn depth img");
            return NULL;
        }
    }
    return NULL;
}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(rclcpp::Time(imu_buf.back()->header.stamp).seconds() > rclcpp::Time(feature_buf.front()->header.stamp).seconds() ))
        {
            //RCLCPP_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(feature_buf.front()->header.stamp).seconds()))
        {
            RCLCPP_WARN(rclcpp::get_logger("rvio_syn"), "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud2::ConstSharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> IMUs;
        while (rclcpp::Time(imu_buf.front()->header.stamp).seconds() < rclcpp::Time(img_msg->header.stamp).seconds() )
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(rclcpp::get_logger("rvio_syn"), "no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void dpt_callback(const sensor_msgs::msg::Image::ConstSharedPtr& dpt_img)
{
    m_dpt_buf.lock();
        dpt_img_buf.push(dpt_img);
    m_dpt_buf.unlock();
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    if (rclcpp::Time(imu_msg->header.stamp).seconds() <= last_imu_t)
    {
        RCLCPP_WARN(rclcpp::get_logger("rvio_syn"), "imu message in disorder!");
        return;
    }
    last_imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
}


void feature_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>, sensor_msgs::msg::PointCloud2::ConstSharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = rclcpp::Time(imu_msg->header.stamp).seconds();
                double img_t = rclcpp::Time(img_msg->header.stamp).seconds();
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    RCLCPP_ASSERT(rclcpp::get_logger("rvio_syn"), dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x*nG;
                    dy = imu_msg->linear_acceleration.y*nG;
                    dz = imu_msg->linear_acceleration.z*nG;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    Vector3d acc(dx, dy, dz);
                    Vector3d gyo(rx, ry, rz);
                    rvio.processIMU(dt, acc, gyo);
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    RCLCPP_ASSERT(rclcpp::get_logger("rvio_syn"), dt_1 >= 0);
                    RCLCPP_ASSERT(rclcpp::get_logger("rvio_syn"), dt_2 >= 0);
                    RCLCPP_ASSERT(rclcpp::get_logger("rvio_syn"), dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x*nG;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y*nG;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z*nG;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    Vector3d acc(dx, dy, dz);
                    Vector3d gyo(rx, ry, rz);
                    rvio.processIMU(dt_1, acc, gyo);
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            RCLCPP_DEBUG(rclcpp::get_logger("rvio_syn"), "processing vision data with stamp %f \n", rclcpp::Time(img_msg->header.stamp).seconds());

            TicToc t_s;
            // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            map<int, vector<pair<int, Eigen::Matrix<double, 10, 1>>>> image;

            pcl::PointCloud<CustomPoint> cloud;
            pcl::fromROSMsg(*img_msg, cloud);

            for (unsigned int i = 0; i < cloud.size(); i++)
            {
                int v = cloud[i].intensity + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = cloud[i].x;
                double y = cloud[i].y;
                // double z = img_msg->points[i].z;
                double z = 0.; // img_msg->points[i].z;
                double p_u = cloud[i].pu;
                double p_v = cloud[i].pv;
                double velocity_x = cloud[i].vx;
                double velocity_y = cloud[i].vy;
                RCLCPP_ASSERT(rclcpp::get_logger("rvio_syn"), z == 1);
                // Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                Eigen::Matrix<double, 10, 1> xyz_uv_velocity;
                double lambda = 0;
                double sig_d = 0;
                double sig_lambda = 0;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, lambda, sig_d, sig_lambda;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }

            // retrieve depth data
            m_dpt_buf.lock();
            sensor_msgs::msg::Image::ConstSharedPtr dpt_ptr = getDptImage(rclcpp::Time(img_msg->header.stamp).seconds());
            m_dpt_buf.unlock();

            bool b_get_floor = false;

            if(dpt_ptr != NULL) {
                if(dpt_ptr->encoding == "16UC1"){
                    sensor_msgs::msg::Image img;
                    img.header = dpt_ptr->header;
                    img.height = dpt_ptr->height;
                    img.width = dpt_ptr->width;
                    img.is_bigendian = dpt_ptr->is_bigendian;
                    img.step = dpt_ptr->step;
                    img.data = dpt_ptr->data;
                    img.encoding = "mono16";

                    if(PUB_DEPTH_IMAGE){
                        // pub_depth_img.publish(img);
                        if(rvio.solver_flag == SolverFlag::NON_LINEAR){ // only publish depth after initialization{
                            pub_depth_img->publish(*dpt_ptr);
                            // RCLCPP_WARN("rvio_syn_node: publish keyframe_depth_image at %lf", dpt_ptr->header.stamp.toSec());
                        }
                    }

                    // ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
                    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
                    if(DEPTH_INTERPOLATE)
                        rvio.associateDepthInterporlate(image, ptr->image);
                    else
                        rvio.associateDepthSimple(image, ptr->image);
                    // if(rvio.solver_flag != INITIAL)
                    //    b_get_floor = rvio.getFloorAndObstacle(ptr->image);
                    // rvio.associateDepthGMM(image, ptr->image);
                }else{
                    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(dpt_ptr, sensor_msgs::image_encodings::MONO16);
                    if(DEPTH_INTERPOLATE)
                        rvio.associateDepthInterporlate(image, ptr->image);
                    else
                        rvio.associateDepthSimple(image, ptr->image);
                    // if(rvio.solver_flag != INITIAL)
                    //    b_get_floor = rvio.getFloorAndObstacle(ptr->image);
                    // rvio.associateDepthGMM(image, ptr->image);
                }
            }

            rvio.processImage_Init(image, rclcpp::Time(img_msg->header.stamp).seconds());

            double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(rvio, header);
            // pubKeyPoses(estimator, header);
            pubCameraPose(rvio, header);
            pubPointCloud(rvio, header);
            pubTF(rvio, header);
            if(rvio.solver_flag != INITIAL){
                if(b_get_floor){
                    // pubFloorPoint(rvio, header);
                }
                // pubNonFloorPoint(rvio, header);
            }
            // pubKeyframe(estimator);
            //RCLCPP_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vins_estimator");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); // Info
    readParameters(node);
    rvio.setParameter();
    RCLCPP_WARN(rclcpp::get_logger("rvio_syn"), "waiting for image and imu...");

    registerPub(node);
    node->declare_parameter("depth_interpolate", DEPTH_INTERPOLATE);
    node->declare_parameter("publish_keyframe_depth_image", PUB_DEPTH_IMAGE);

    node->get_parameter("depth_interpolate", DEPTH_INTERPOLATE);
    node->get_parameter("publish_keyframe_depth_image", PUB_DEPTH_IMAGE);

    if(PUB_DEPTH_IMAGE)
        pub_depth_img = node->create_publisher<sensor_msgs::msg::Image>("/keyframe_depth_image", 2000);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 2000, imu_callback);
    auto sub_image = node->create_subscription<sensor_msgs::msg::PointCloud2>("/feature_tracker/feature", 2000, feature_callback);
    auto sub_dpt = node->create_subscription<sensor_msgs::msg::Image>(DPT_IMG_TOPIC, 2000, dpt_callback);
    // ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    // ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process};
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
