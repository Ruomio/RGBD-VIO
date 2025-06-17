#include <pcl/conversions.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/sensor_msgs/msg/image_encodings.h>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>

#include "pcl_cloud.h"
#include "feature_tracker.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_img;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_match;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> pub_restart;

bool PUB_IMAGE = false;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_color_img;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

// vector<string> CAM_NAMES; // TODO: move it to parameters

void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
        last_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
        return;
    }
    // detect unstable camera stream
    if (rclcpp::Time(img_msg->header.stamp).seconds() - last_image_time > 1.0 || rclcpp::Time(img_msg->header.stamp).seconds() < last_image_time)
    {
        RCLCPP_WARN(rclcpp::get_logger("feature_tracker"), "image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
    // frequency control
    if (round(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time)) <= FREQ)
    {
        RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "feature_node: first_image_time: %lf current_time: %lf now publish", first_image_time, rclcpp::Time(img_msg->header.stamp).seconds());
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (rclcpp::Time(img_msg->header.stamp).seconds() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = rclcpp::Time(img_msg->header.stamp).seconds();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    cv::Mat ret_img;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	ret_img = ptr->image.clone();
    }else if(img_msg->encoding == "8UC3"){
	sensor_msgs::msg::Image img;
	img.header = img_msg->header;
	img.height = img_msg->height;
	img.width = img_msg->width;
	img.is_bigendian = img_msg->is_bigendian;
	img.step = img_msg->step;
	img.data = img_msg->data;
	img.encoding = "bgr8";
	ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	ret_img = ptr->image.clone();
	cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }else{
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
	ret_img = ptr->image.clone();
    }
    cv::Mat show_img = ret_img; // ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("feature_tracker"), "processing camera %d", i);
        if (i != 1)
            // trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
	    trackerData[i].readImage(ret_img, rclcpp::Time(img_msg->header.stamp).seconds());
        else
        {
		  trackerData[i].cur_img = ret_img;
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            // if (j != 1 || !STEREO_TRACK)
            if(j != 1)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud2::SharedPtr feature_points(new sensor_msgs::msg::PointCloud2);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        pcl::PointCloud<CustomPoint> point;


        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    // geometry_msgs::msg::Point32 p;
                    CustomPoint p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    p.intensity = p_id * NUM_OF_CAM + i;
                    p.pu = cur_pts[i].x;
                    p.pv = cur_pts[j].y;
                    p.vx = pts_velocity[i].x;
                    p.vy = pts_velocity[j].y;
                    point.push_back(p);

                    // feature_points->points.push_back(p);
                    // id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    // u_of_point.values.push_back(cur_pts[j].x);
                    // v_of_point.values.push_back(cur_pts[j].y);
                    // velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    // velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        // feature_points->channels.push_back(id_of_point);
        // feature_points->channels.push_back(u_of_point);
        // feature_points->channels.push_back(v_of_point);
        // feature_points->channels.push_back(velocity_x_of_point);
        // feature_points->channels.push_back(velocity_y_of_point);

        pcl::toROSMsg(point, *feature_points);
        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";
        RCLCPP_INFO(rclcpp::get_logger("feature_tracker"), "publish %f, at %f with features %d", rclcpp::Time(feature_points->header.stamp).seconds(), rclcpp::Clock().now().seconds(), (int)feature_points->data.size());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img->publish(*feature_points);

        // publish the
        if(PUB_IMAGE){
            // sensor_msgs::Image img;
            // img.header = img_msg->header;
            // img.height = img_msg->height;
            // img.width = img_msg->width;
            // img.is_bigendian = img_msg->is_bigendian;
            // img.step = img_msg->step;
            // img.data = show_img.data;
            // img.encoding = "mono8";
            // pub_color_img.publish(img);
            pub_color_img->publish(*img_msg);
        }


        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * IMAGE_ROW, (i + 1) * IMAGE_ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match->publish(*(ptr->toImageMsg()));
        }

    }
    // RCLCPP_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("feature_tracker");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);  // Debug Info
    readParameters(node);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES);

    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, 100, img_callback);

    pub_img = node->create_publisher<sensor_msgs::msg::PointCloud2>("feature", 1000);
    pub_match = node->create_publisher<sensor_msgs::msg::Image>("feature_img",1000);
    pub_restart = node->create_publisher<std_msgs::msg::Bool>("restart",1000);

    // n.param("publish_keyframe_color_image", PUB_IMAGE, PUB_IMAGE);
    node->declare_parameter("publish_keyframe_color_image", PUB_IMAGE);
    node->get_parameter("publish_keyframe_color_image", PUB_IMAGE);

    if(PUB_IMAGE)
        pub_color_img = node->create_publisher<sensor_msgs::msg::Image>("/keyframe_color_image", 1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    rclcpp::spin(node);
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
