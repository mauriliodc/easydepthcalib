#ifndef PCC_H
#define PCC_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace Eigen;
using namespace std;


struct cloudData{
    float x;
    float y;
    float z;
};


class pointCloud{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pointCloud(cv::Mat depthImage,Matrix3f K);
    pointCloud(pcl::PointCloud<pcl::PointXYZ>* pcl);
    pointCloud();
    ~pointCloud();

    pcl::PointCloud<pcl::PointXYZ>* pclCloud();
    void T( Eigen::Isometry3f t);
    std::vector<cloudData> cloud;
    cv::Mat depthImage;
    Matrix3f K;
};

#endif
