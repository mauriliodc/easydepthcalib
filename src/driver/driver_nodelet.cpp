#include "driver_nodelet.h"
#include <pluginlib/class_list_macros.h>

#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */

using namespace std;
using namespace ros;

namespace easydepthcalib
{
    DriverNodelet::DriverNodelet() : multiplier{NULL} {}
    DriverNodelet::~DriverNodelet()
    {
        if (multiplier)
        {
            multiplier->clear();
            delete multiplier;
            delete image_transport;
        }
    }

    void DriverNodelet::onInit()
    {
        // Get params
        getPrivateNodeHandle().param<string>("topic_sub", topic_sub, "");
        getPrivateNodeHandle().param<string>("topic_pub", topic_pub, "");
        getPrivateNodeHandle().param<string>("calib_lut", calib_lut, "");

        // info
        if (topic_sub == "" || topic_pub == "" || calib_lut == "")
        {
            cout << BOLDYELLOW << endl;
            cout << "======================================================" << endl;
            cout << "=========== EASYDEPTHCALIB ONLINE DRIVER =============" << endl;
            cout << "======================================================" << endl << endl << RESET;

            cout << "This is the calibration driver and it's intended to use in realtime, params:" << endl;
            cout << BOLDYELLOW << "_topic_pub " << RESET << "-- input depth image namespace (eg. /camera/depth, where /camera/depth/image_raw" << endl
                 << "and /camera/depth/camera_info will be subscribed automaticly." << endl;
            cout << BOLDYELLOW << "_topic_sub " << RESET << "-- output calibrated depth image namespace (eg. /camera/depth_calib, where /camera/depth_calib/image_raw" << endl
                 << "and /camera/depth_calib/camera_info will be published automaticly." << endl;
            cout << BOLDYELLOW << "_calib_lut " << RESET << "-- calibration LUT file" << endl << endl;

            exit(-1);
        }

        // What are the subscribtion topics?
        if (topic_sub.at(topic_sub.size()-1) == '/')
        {
            image_raw_topic_sub = topic_sub + "image_raw";
            camera_info_topic_sub = topic_sub + "camera_info";
        }
        else
        {
            image_raw_topic_sub = topic_sub + "/image_raw";
            camera_info_topic_sub = topic_sub + "/camera_info";
        }

        // What are the publication topics?
        if (topic_pub.at(topic_pub.size()-1) == '/')
        {
            image_raw_topic_pub = topic_pub + "image_raw";
            camera_info_topic_pub = topic_pub + "camera_info";
        }
        else
        {
            image_raw_topic_pub = topic_pub + "/image_raw";
            camera_info_topic_pub = topic_pub + "/camera_info";
        }

        // Image pub&sub
        image_transport = new image_transport::ImageTransport(getPrivateNodeHandle());
        pub_image_raw = image_transport->advertise(image_raw_topic_pub,  1,
                                                   boost::bind(&DriverNodelet::connectCb, this),
                                                   boost::bind(&DriverNodelet::disconnectCb, this));

        // CameraInfo pub&sub
        pub_camera_info = getPrivateNodeHandle().advertise<sensor_msgs::CameraInfo>(camera_info_topic_pub, 1);
        sub_camera_info = getPrivateNodeHandle().subscribe(camera_info_topic_sub, 1, &DriverNodelet::cameraInfoCallback, this);

        //Calib file
        multiplier = new calibrationMatrix(const_cast<char*>(calib_lut.c_str()));
    }
    void DriverNodelet::connectCb()
    {
        if (!sub_image_raw && pub_image_raw.getNumSubscribers()>0)
        {
            NODELET_INFO("Connecting to camera topic.");
            sub_image_raw = image_transport->subscribe(image_raw_topic_sub, 1, &DriverNodelet::imageCallback, this);
        }
    }
    void DriverNodelet::disconnectCb()
    {
        if (pub_image_raw.getNumSubscribers() == 0)
        {
              NODELET_INFO("Unsubscribing from camera topic.");
              sub_image_raw.shutdown();
        }
    }

    void DriverNodelet::imageCallback(const sensor_msgs::ImageConstPtr &image_ptr)
    {
        // To cvbridge
        cv_bridge::CvImagePtr msg_in = cv_bridge::toCvCopy(image_ptr);

        // mat
        cv::Mat image;
        msg_in->image.copyTo(image);

        // Multiplication
        int cols=image.cols;
        int rows=image.rows;
        unsigned short v;
        cv::Point p;
        for(int i=0;i<cols;i++){
            for(int j=0;j<rows;j++){
                p.x=i;
                p.y=j;
                v=((float)image.at<unsigned short>(p));
                v*=multiplier->cell(p.y,p.x,v);
                image.at<unsigned short>(p)=(unsigned short)v;
            }
        }

        // Msgs
        cv_bridge::CvImage msg_out;
        msg_out.header   = image_ptr->header;
        msg_out.encoding = "16UC1";
        msg_out.image = image;
        pub_image_raw.publish(msg_out.toImageMsg());
    }
    void DriverNodelet::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr)
    {
        pub_camera_info.publish(camera_info_ptr);
    }
}

PLUGINLIB_DECLARE_CLASS(easydepthcalib, DriverNodelet, easydepthcalib::DriverNodelet, nodelet::Nodelet);
