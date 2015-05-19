
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#include "pointCloud.h"
#include "planeCloud.h"
#include "calibration.h"

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc<3){
        std::cout<<"This node is intend to use offline"<<std::endl<<"provide a logfile (produced with the dumpernode) and the output calibration file you want"<<std::endl;
        std::cout<<"example:"<<std::endl<<"offlineCalibration <input log filename> <output calibration filename>"<<std::endl;
        return 0;
    }
    std::fstream log;
    log.open(argv[1]);
    string topic;
    int height;
    int width;
    log >> topic;
    log >> height;
    log >> width;
    Matrix3f k;
    log >> k(0,0);
    log >> k(0,1);
    log >> k(0,2);
    log >> k(1,0);
    log >> k(1,1);
    //k(1,1)*=-1;
    log >> k(1,2);
    log >> k(2,0);
    log >> k(2,1);
    log >> k(2,2);
    //k.setIdentity();
    std::cout<<k.inverse()<<std::endl;
    string filename;
    calibrationMatrix c(640,480,7000,4,64);
    while(!log.eof()){
        log>>filename;
        std::cout<< "opening "<<filename<<"\r"<<std::flush;
        cv::Mat data = cv::imread(filename,cv::IMREAD_ANYDEPTH);

        pointCloud p(data,k);
        //VOXELIZER
        pcl::PointCloud<pcl::PointXYZ>* pcl;
        pcl=p.pclCloud();
        calibration::voxelize(pcl,pcl,20);
        pointCloud p2(pcl);
        //CENTER SQUARE CLOUD
        pcl::PointCloud<pcl::PointXYZ>* square= new pcl::PointCloud<pcl::PointXYZ>();
        calibration::computeCenterSquareCloud(data,k,square,100,100);
        pointCloud p3(square);
        //CENTER PLANE
        Eigen::Vector4f centerModel;
        calibration::computerCenterPlane(square,centerModel,40);
        std::cout.flush();
        planeCloud centerPlaneCloud;
        centerPlaneCloud.model=centerModel;
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*square,center);
        //std::cout<<"CENTROID DISTANCE: "<<center[2]<<" ( "<<center.transpose()<<" )"<<std::endl;
        centerPlaneCloud.com=center.head(3);
        //NORMALS
        pcl::PointCloud<pcl::Normal>* normals= new pcl::PointCloud<pcl::Normal>();
        calibration::computeNormals(p2.pclCloud(),normals,100);
        //REJECTION
        Eigen::Vector3f ref;
        std::vector<bool> valid;
        ref<<centerModel[0],centerModel[1],centerModel[2];
        pcl::PointCloud<pcl::PointXYZ> outFromNormalRejection;
        pcl::PointCloud<pcl::PointXYZ>* tmpCloud =p2.pclCloud();
        calibration::pointrejection(&ref,0.7f,tmpCloud,normals,&outFromNormalRejection,&valid);

        //pointCloud outFromNormalRejectionCloud(&outFromNormalRejection);
        //ERROR PER POINT
        pcl::PointCloud<pcl::PointXYZ> error;
        calibration::computeErrorPerPoint(tmpCloud,&error,center.head(3),ref,&valid);
        //std::cout<<"error cloud has "<<error.size()<<" points"<<std::endl;
        //std::cout<<"voxel cloud has "<<p2.pclCloud()->size()<<" points"<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>* cloudToCalibrate=p2.pclCloud();
        //COMPUTE CALIBRATION MATRIX
        std::cout.flush();
        calibration::computeCalibrationMatrix(*cloudToCalibrate,error,k,&valid,c);
        std::cout.flush();
        //CALIBRATE POINT CLOUD
        pcl::PointCloud<pcl::PointXYZ> fixedCloud;
        calibration::calibratePointCloudWithMultipliers(*cloudToCalibrate,fixedCloud,c,k);
        std::cout.flush();

        error.clear();
        valid.clear();
        error.clear();
        cloudToCalibrate->clear();
        delete cloudToCalibrate;
        tmpCloud->clear();
        delete tmpCloud;
        p2.cloud.clear();
        normals->clear();
        outFromNormalRejection.clear();
        valid.clear();
        delete normals;
        delete square;
        pcl->clear();
        delete pcl;
    }
    std::cout<<std::endl<<"saving "<<std::endl;
    c.serialize(argv[2]);
    char nn[50];
    std::cout<<"saving nn"<<std::endl;
    sprintf(nn,"NN_%s",argv[2]);
    c.serializeNN(nn);
}
