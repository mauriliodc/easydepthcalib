#include "driver_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(DriverNodelet, nodelet::Nodelet)

using namespace std;
using namespace ros;

DriverNodelet::DriverNodelet() {}
DriverNodelet::~DriverNodelet()
{
    delete driver;
}

void DriverNodelet::onInit()
{
    ros::NodeHandle nh("~");
    driver = new Driver(nh);
}
