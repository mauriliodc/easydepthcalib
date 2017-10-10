#ifndef DRIVER_H
#define DRIVER_H

// Ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/opencv.hpp>

// easydepthcalib
#include "calibrationMatrix.h"


class Driver
{

    // ================================
    // ======== PUBLIC METHODS ========
    // ================================

public:

    Driver(ros::NodeHandle n);

    ~Driver();

protected:

    // ==================================
    // ========= CALIB VARIABLES ========
    // ==================================

    calibrationMatrix* multiplier;

    // ==================================
    // ========= PARAM VARIABLES ========
    // ==================================

    std::string topic_sub;
    std::string topic_pub;
    std::string calib_lut;

    std::string image_raw_topic_sub;
    std::string camera_info_topic_sub;

    std::string image_raw_topic_pub;
    std::string camera_info_topic_pub;

    // ==================================
    // ========= ROS SUBSCRIBERS ========
    // ==================================

    image_transport::ImageTransport* image_transport;
    image_transport::Subscriber sub_image_raw;
    ros::Subscriber sub_camera_info;

    // ==================================
    // ======== ROS PUBLISHERS ==========
    // ==================================

    image_transport::Publisher pub_image_raw;
    ros::Publisher pub_camera_info;

    // ==================================
    // ======== ROS CALLBACKS ===========
    // ==================================

    void imageCallback(const sensor_msgs::ImageConstPtr &imgPtr);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr);
};

#endif //DRIVER_H
