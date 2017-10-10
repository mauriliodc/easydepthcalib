#include "driver.h"

using namespace std;
using namespace ros;

Driver::Driver(ros::NodeHandle n) : multiplier{NULL}
{
    // Get params
    n.param("topic_sub", topic_sub);
    n.param("topic_pub", topic_pub);
    n.param("calib_lut", calib_lut);

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
    image_transport = new image_transport::ImageTransport(n);
    pub_image_raw = image_transport->advertise(image_raw_topic_pub, 1);
    sub_image_raw = image_transport->subscribe(image_raw_topic_sub, 1, &Driver::imageCallback, this);

    // CameraInfo pub&sub
    pub_camera_info = n.advertise<sensor_msgs::CameraInfo>(camera_info_topic_pub, 1);
    sub_camera_info = n.subscribe(camera_info_topic_sub, 1, &Driver::cameraInfoCallback, this);

    //Calib file
    multiplier = new calibrationMatrix(const_cast<char*>(calib_lut.c_str()));
}

Driver::~Driver()
{
    if (multiplier)
    {
        multiplier->clear();
        delete multiplier;
        delete image_transport;
    }
}

void Driver::imageCallback(const sensor_msgs::ImageConstPtr &image_ptr)
{
    // To cvbridge
    cv_bridge::CvImagePtr cvImageFromROS =cv_bridge::toCvCopy(image_ptr);

    // mat
    cv::Mat image;
    cvImageFromROS->image.copyTo(image);

    // Msgs
    cv_bridge::CvImage out_msg;
    out_msg.header   = image_ptr->header;
    out_msg.encoding = "mono16";

    // Multiplication
    int cols=image.cols;
    int rows=image.rows;
    ushort v;
    cv::Point p;
    for(int i=0;i<cols;i++){
        for(int j=0;j<rows;j++){
            p.x=i;
            p.y=j;
            v=((float)image.at<ushort>(p));
            v*=multiplier->cell(p.y,p.x,v);
            image.at<ushort>(p)=(ushort)v;
        }
    }

    // Publish
    out_msg.image = image;
    pub_image_raw.publish(out_msg.toImageMsg());
}

void Driver::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info_ptr)
{
    pub_camera_info.publish(camera_info_ptr);
}
