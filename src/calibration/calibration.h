#ifndef CALIB_H
#define CALIB_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "calibrationMatrix.h"
#include <iostream>
using namespace pcl;

class calibration{
public:
    inline calibration(){}
    inline ~calibration(){}
    static void voxelize(pcl::PointCloud<pcl::PointXYZ>* cloud,pcl::PointCloud<pcl::PointXYZ>* outCloud, float voxelLeaf);
    static void computeCenterSquareCloud(cv::Mat& depthImage,Eigen::Matrix3f k, pcl::PointCloud<pcl::PointXYZ>* outCloud, int width, int height);
    static void computerCenterPlane(pcl::PointCloud<pcl::PointXYZ>* inCloud,Eigen::Vector4f& model, int distanceThreshold);
    static void computeNormals(pcl::PointCloud<pcl::PointXYZ>* inCloud, pcl::PointCloud<pcl::Normal>* normals, float searchRadius);
    static void pointrejection(Eigen::Vector3f* referenceNormal,float rejectionValue,pcl::PointCloud<pcl::PointXYZ>* inCloud,pcl::PointCloud<pcl::Normal>* normals,pcl::PointCloud<pcl::PointXYZ>* outCloud, std::vector<bool>* validIndeces );
    //OUTPUT errorCloud
    static void computeErrorPerPoint(pcl::PointCloud<pcl::PointXYZ>* inCloud,
                                     pcl::PointCloud<pcl::PointXYZ>* errorCloud,
                                     Eigen::Vector3f planeOrigin,
                                     Eigen::Vector3f planeCoefficient,
                                     std::vector<bool>* validIndeces );

    static void computeCalibrationMatrix(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                         pcl::PointCloud<pcl::PointXYZ> &errorCloud,
                                         Eigen::Matrix3f k,
                                         std::vector<bool>* validIndeces,
                                         calibrationMatrix &cM);
    static void calibratePointCloudWithMultipliers(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                                   pcl::PointCloud<pcl::PointXYZ> &outCloud,
                                                   calibrationMatrix &cM,
                                                   Eigen::Matrix3f k);
};


#endif
