#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sstream>
#include <iostream>
#include <fstream>


using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
using namespace message_filters;
static const char WINDOW1[] = "Image window_1";

std::string imageTopic;
std::string cameraInfoTopic;
std::string logFile;
std::string outputDirectory;
std::ofstream outfile;
bool gotCameraInfo=false;
int imageNumber = 0;

void imageCb(const sensor_msgs::ImageConstPtr& msg_1)
{
    if(!gotCameraInfo) return;
    cv_bridge::CvImagePtr cv_ptr_1;
    cv_ptr_1 = cv_bridge::toCvCopy(msg_1,sensor_msgs::image_encodings::TYPE_16UC1);
    char filename[100];

    //save image 1
    sprintf(filename,"%s/%05d.pgm",outputDirectory.c_str(),imageNumber);
    std::cout<< "saving "<< filename<<std::endl;
    cv::imwrite(filename,cv_ptr_1->image);
    outfile<<filename<<std::endl;
    //increment stuff
    imageNumber++;
    //show stuff
    //cv::imshow(WINDOW1, cv_ptr_1->image);
    //cv::waitKey(3);
}

void camerainfoCb(const sensor_msgs::CameraInfoConstPtr&  info){
    if(gotCameraInfo) return;
    std::cout<< "got camera info"<<std::endl;
    outfile<< imageTopic<< " ";
    outfile<< info->height<< " "<<info->width<<" ";
    outfile<<    info->K.at(0)<<" "<< info->K.at(1)<<" "<< info->K.at(2)<<" "
              << info->K.at(3)<<" "<< info->K.at(4)<<" "<< info->K.at(5)<<" "
              << info->K.at(6)<<" "<< info->K.at(7)<<" "<< info->K.at(8)<<std::endl;
    gotCameraInfo=true;
}

void banner(){
    std::cout<<"DepthDumperNode is intented to use to dump the depth images and then launch on such data the calibration procedure"<<std::endl;
    std::cout<<"It's basically a best practise to do like that to avoid all possible issues that could arise from running in realtime"<<std::endl;
    std::cout<<"Usage:"<<std::endl;
    std::cout<<"rosrun easydepthcalibration depthdumpernode _imageTopic:=/camera/depth/image_raw _cameraInfoTopic:=/camera/depth/camera_info _logFile:=calibation_file.mal _outputDirectory_=distortedImages"<<std::endl;
}

int main(int argc, char** argv)
{
    if(argc<2){
        banner();
        exit(1);
    }

    ros::init(argc, argv, "easyDepthCalibDumper");
    ros::NodeHandle nh_("~");
    nh_.param("imageTopic",imageTopic,std::string("/camera/depth/image_raw"));
    nh_.param("cameraInfoTopic",cameraInfoTopic,std::string("/camera/depth/camera_info"));
    nh_.param("logFile",logFile,std::string("out.mal"));
    nh_.param("outputDirectory",outputDirectory,std::string("."));
    
    std::cout<<"Creating [logFile] to "<<outputDirectory<<logFile<<std::endl;
    char logfilepath[100];
    mkdir(outputDirectory.c_str(), 0700);
    sprintf(logfilepath,"%s",outputDirectory.c_str());
    outfile.open(logFile.c_str(), std::ios::out);
    std::cout<<"Dumping data to "<< outputDirectory<<std::endl;
    std::cout<<"Please remember to set your sensor at lowest resolution (intepolation is useless and is bandwidth consuming)"<<std::endl;
    ros::Subscriber sub1=nh_.subscribe(imageTopic, 10, imageCb);
    ros::Subscriber sub2=nh_.subscribe(cameraInfoTopic, 10, camerainfoCb);
    ros::spin();
    return 0;
}
