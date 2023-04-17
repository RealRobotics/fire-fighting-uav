#ifndef FIRE_DETECTOR_H
#define FIRE_DETECTOR_H

#include <iostream>
#include <numeric>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <uav_msgs/WallExtremities.h>
#include <uav_msgs/ClosestObject.h>
#include "thermal_camera.h"

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 360
// Set both of these to 0 when running properly.
#define SAVE_IMAGES 1
#define SHOW_IMAGES 0

#if SAVE_IMAGES
#include <opencv2/videoio.hpp>
#endif

class fire_detector
{
public:
    fire_detector(ros::NodeHandle* nodehandle);
    ~fire_detector();

    void run();

private:
    void detect_fire(cv::Mat thermal_img, rs2::depth_frame depth, rs2_intrinsics intr);
    void obtain_extremities_distances(rs2::depth_frame depth);
    void obtain_closest_obj_info(rs2::depth_frame depth, rs2_intrinsics intr);
    void publish_imu_data(rs2::motion_frame gyro_frame, rs2::motion_frame accel_frame);

    mlx_thermal_cam thermal_cam;

    ros::NodeHandle nh;

    ros::Publisher pose_pub;
    ros::Publisher extremities_pub;
    ros::Publisher closest_obj_pub;
    ros::Publisher imu_pub;

    rs2::pipeline pipeline;

    rs2::colorizer colorizer;

    rs2::pointcloud pcl;
    rs2::points pcl_points;

    rs2::threshold_filter thr_filter;

#if SAVE_IMAGES
    void openFiles();
    cv::VideoWriter video_thermal_raw_;
    cv::VideoWriter video_thermal_cropped_;
    cv::VideoWriter video_realsense_depth_;
#endif

};

#endif
