#include <ros/ros.h>
#include "../shared/CalibrationMatrix.h"
#include <ecl/threads/thread.hpp>
#include <sensor_msgs/Image.h>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>
class xtionDriver{
public:
    inline xtionDriver(ros::NodeHandle n) : multiplier("prova.txt"),
                                            shutdown_required(false),
                                            thread(&xtionDriver::spin, *this),
                                            it(n),itDIFF(n){

        std::cout<<"Freezing multiplier matrix...";
        multiplier.syncToFloat();
        std::cout<<"done!"<<std::endl;
        std::cout.flush();
        pub = it.advertise("/camera/malcom", 10);
        pubDIFF = itDIFF.advertise("/camera/malcom_diff", 10);
    }

    inline ~xtionDriver(){}

    image_transport::ImageTransport it;
    image_transport::ImageTransport itDIFF;
    image_transport::Publisher pubDIFF;
    image_transport::Publisher pub;

    ecl::Thread thread;
    bool shutdown_required;
    void callback(const sensor_msgs::ImageConstPtr &imgPtr);
    void spin();
    CalibrationMatrix  multiplier;

    private:
    cv::Mat image;
    cv::Mat original;
    cv_bridge::CvImagePtr cvImageFromROS;
    cv::Point p;
    ushort v;
    cv_bridge::CvImage out_msg;
    cv_bridge::CvImage out_msg_DIFF;

};
