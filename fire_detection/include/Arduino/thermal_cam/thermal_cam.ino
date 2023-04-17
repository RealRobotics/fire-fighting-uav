#include "fire_detector.h"


fire_detector::fire_detector(ros::NodeHandle *nodehandle)
    : nh(*nodehandle)
{
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("fire_poses", 10);
    extremities_pub = nh.advertise<uav_msgs::WallExtremities>("wall_extremities", 10);
    closest_obj_pub = nh.advertise<uav_msgs::ClosestObject>("closest_obj", 10);
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 15);
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    rs2::pipeline_profile selection = pipeline.start(cfg);
    rs2::device selected_device = selection.get_device();

    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.0f); // Enable emitter
        ROS_INFO("Enabled emitter");
    }

    ROS_INFO("Fire detection node initialised, starting...");
}

fire_detector::~fire_detector()
{

}

void fire_detector::run()
{
    ros::Rate rate(32);

#if SAVE_IMAGES
    openFiles();
#endif

    while(ros::ok())
    {
        // wait for frames to become available

        rs2::frameset data = pipeline.wait_for_frames();

        // frames containing IMU data

        //rs2::motion_frame gyro_frame = data.first_or_default(RS2_STREAM_GYRO);
        //rs2::motion_frame accel_frame = data.first_or_default(RS2_STREAM_ACCEL);

        //publish_imu_data(gyro_frame, accel_frame);


        rs2::depth_frame depth = data.get_depth_frame();

        rs2::colorizer colorizer;
        rs2::frame depth_frame = data.get_depth_frame();
        depth_frame = colorizer.process(depth_frame);

        cv::Mat cv_depth(cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

#if SAVE_IMAGES
        video_realsense_depth_ << cv_depth;
#endif

#if SHOW_IMAGES
        cv::imshow("cv_depth", cv_depth);
#endif
        rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data

        cv::Mat thermal_img = thermal_cam.get_frame();

        obtain_extremities_distances(depth);
        detect_fire(thermal_img, depth_frame, intr);
        obtain_closest_obj_info(depth_frame, intr);

        cv::waitKey(1);

        ros::spinOnce();
        rate.sleep();
    }
}

void fire_detector::detect_fire(cv::Mat thermal_img, rs2::depth_frame depth, rs2_intrinsics intr)
{
    cv::resize(thermal_img, thermal_img, cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT));

    cv::Rect roi(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
    cv::Mat cropped_img = thermal_img(roi);

#if SAVE_IMAGES
    video_thermal_raw_ << thermal_img;
    video_thermal_cropped_ << cropped_img;
#endif

#if SHOW_IMAGES
    cv::imshow("uncropped thermal", thermal_img);
    cv::imshow("cropped thermal", cropped_img);
    // cv::imshow("realsense depth", cv_depth);
#endif

    int lower_temp_limit = 70;
    cv::Mat mask;
    cv::inRange(cropped_img, lower_temp_limit, 255, mask);
    cropped_img = cropped_img/2;

    double avg_temp = cv::sum(cropped_img)[0]/(cropped_img.size().width*cropped_img.size().height);
/*
    cv::Mat temp_mask;
    cv::inRange(cropped_img, avg_temp+5, 255, temp_mask);

    cv::inRange(temp_mask, 1, 255, temp_mask);

    cv::Mat temp_mask2;
    cv::bitwise_and(temp_mask, cv_depth, temp_mask2);

    cv::Mat temp_mask3;
    cv::bitwise_and(temp_mask2, cropped_img, temp_mask3);

    cv::Mat final_img;
    final_img = temp_mask2 + temp_mask3;

    cv::inRange(final_img, 70, 255, final_img);

    cv::imshow("binary img", final_img);


    // detected fires will be the contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(final_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point3d> fire_positions;

    // loop through all detected contours and save their positions in a vector
    for (size_t i = 0; i < contours.size(); i++)
    {
        float center_pixel[2];
        cv::Moments M = cv::moments(contours[i]);
        center_pixel[0] = float(M.m10/M.m00);
        center_pixel[1] = float(M.m01/M.m00);

        float point[3];
        float dist = depth.get_distance(center_pixel[0], center_pixel[1]);
        rs2_deproject_pixel_to_point(point, &intr, center_pixel, dist);

        cv::Point3d coordinates = {point[2], point[0]*-1, point[1]*-1}; // convert to ROS standard
        fire_positions.push_back(coordinates);
    }

    geometry_msgs::PoseArray fire_poses_msg;

    // loop through all poses and save them in the PoseArray msg
    for (size_t i = 0; i < fire_positions.size(); i++)
    {
        geometry_msgs::Pose pose;

        pose.position.x = fire_positions[i].x;
        pose.position.y = fire_positions[i].y;
        pose.position.z = fire_positions[i].z;

        fire_poses_msg.poses.push_back(pose);
    }

    fire_poses_msg.header.frame_id = "D435i";
    fire_poses_msg.header.seq += 1;
    fire_poses_msg.header.stamp = ros::Time::now();

    pose_pub.publish(fire_poses_msg);*/
}

void fire_detector::obtain_extremities_distances(rs2::depth_frame depth)
{
    std::vector<double> left_distances;

    // obtain points starting from pixel column 2 in depth img
    for (int i = 0; i <= 10; i++)
    {
        int point[2] = {i+2, DEPTH_HEIGHT/2};

        double distance = depth.get_distance(point[0], point[1]);

        left_distances.push_back(distance);
    }

    float left_average = std::accumulate(left_distances.begin(), left_distances.end(), 0.0)/left_distances.size();


    std::vector<double> right_distances;

    // obtain points starting from the last pixel column - 2 in depth img
    for (int i = 0; i <= 10; i++)
    {
        int point[2] = {DEPTH_WIDTH - 2, DEPTH_HEIGHT/2};

        double distance = depth.get_distance(point[0], point[1]);

        right_distances.push_back(distance);
    }

    float right_average = std::accumulate(right_distances.begin(), right_distances.end(), 0.0)/right_distances.size();


    uav_msgs::WallExtremities wall_extremities_msg;

    wall_extremities_msg.Header.frame_id = "D435i";
    wall_extremities_msg.Header.seq += 1;
    wall_extremities_msg.Header.stamp = ros::Time::now();

    wall_extremities_msg.left_wall_dist = left_average;
    wall_extremities_msg.right_wall_dist = right_average;

    extremities_pub.publish(wall_extremities_msg);
}

void fire_detector::obtain_closest_obj_info(rs2::depth_frame depth, rs2_intrinsics intr)
{
    float lowest_distance = 10;
    float pixel[2] = {0, 0};

    // loop through all pixels in image to find the closest object
    for (int i = 0; i < DEPTH_WIDTH; i++) {
        for (int j = 0; j < DEPTH_HEIGHT; j++) {
            int point[2] = {i, j};
            float pixel_dist = depth.get_distance(point[0], point[1]);

            if (std::isfinite(pixel_dist) && pixel_dist != 0 && pixel_dist < lowest_distance) {
                lowest_distance = pixel_dist;
                pixel[0] = i; pixel[1] = j;
            }
        }
    }

    // obtain 3D coordinates of the object

    float point[3];
    rs2_deproject_pixel_to_point(point, &intr, pixel, lowest_distance);

    cv::Point3d coordinates = {point[2], point[0]*-1, point[1]*-1}; // convert to ROS standard

    uav_msgs::ClosestObject closest_obj_msg;

    closest_obj_msg.header.frame_id = "D435i";
    closest_obj_msg.header.seq += 1;
    closest_obj_msg.header.stamp = ros::Time::now();

    // direct distance to the object
    double euclidean_dist = sqrt( (coordinates.x*coordinates.x) + (coordinates.y*coordinates.y) + (coordinates.z*coordinates.z) );
    closest_obj_msg.euclidean_distance = euclidean_dist;

    closest_obj_msg.pose.position.x = coordinates.x;
    closest_obj_msg.pose.position.y = coordinates.y;
    closest_obj_msg.pose.position.z = coordinates.z;

    closest_obj_pub.publish(closest_obj_msg);
}

void fire_detector::publish_imu_data(rs2::motion_frame gyro_frame, rs2::motion_frame accel_frame)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imu_msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    imu_msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};


    // Coordinate system is the ROS standard, X fwd & Z up

    imu_msg.header.frame_id = "/D435i/imu";
    imu_msg.header.seq += 1;
    imu_msg.header.stamp = ros::Time::now();

    rs2_vector gyro_data = gyro_frame.get_motion_data();

    imu_msg.angular_velocity.x = gyro_data.z;
    imu_msg.angular_velocity.y = gyro_data.x * -1;
    imu_msg.angular_velocity.z = gyro_data.y * -1;

    rs2_vector accel_data = accel_frame.get_motion_data();

    imu_msg.linear_acceleration.x = accel_data.z;
    imu_msg.linear_acceleration.y = accel_data.x * -1;
    imu_msg.linear_acceleration.z = accel_data.y * -1;

    imu_pub.publish(imu_msg);
}

#if SAVE_IMAGES
void fire_detector::openFiles() {
    int four_cc = cv::VideoWriter::fourcc('M','J','P','G');
    double frames_per_second = 32.0;
    cv::Size frame_size = cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT);
    std::string name = "realsense_depth";
    video_realsense_depth_.open(name, four_cc, frames_per_second, frame_size, true);
    if (!video_realsense_depth_.isOpened())
    {
        ROS_ERROR("Could not open %s!", name.c_str());
        exit(-1);
    }
    name = "thermal_raw";
    video_thermal_raw_.open(name, four_cc, frames_per_second, frame_size, true);
    if (!video_thermal_raw_.isOpened())
    {
        ROS_ERROR("Could not open %s!", name.c_str());
        exit(-1);
    }
    name = "thermal_cropped";
    video_thermal_cropped_.open(name, four_cc, frames_per_second, frame_size, true);
    if (!video_thermal_cropped_.isOpened())
    {
        ROS_ERROR("Could not open %s!", name.c_str());
        exit(-1);
    }
}
#endif
