#ifndef DRIVER_NODELET_H
#define DRIVER_NODELET_H

// Ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/opencv.hpp>

// easydepthcalib
#include "calibrationMatrix.h"

namespace easydepthcalib
{
    class DriverNodelet : public nodelet::Nodelet
    {

        // ================================
        // ======== PUBLIC METHODS ========
        // ================================

    public:

        DriverNodelet();

        ~DriverNodelet();

    private:

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

        /*!
          * @brief Serves as a psuedo constructor for nodelets.
          *
          * This function needs to do the MINIMUM amount of work
          * to get the nodelet running.  Nodelets should not call
          * blocking functions here for a significant period of time.
          */
        virtual void onInit();

        void connectCb();
        void disconnectCb();

        void imageCallback(const sensor_msgs::ImageConstPtr &imgPtr);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr);
    };
}

#endif //DRIVER_NODELET_H
