#include "pointCloud.h"



pointCloud::pointCloud(cv::Mat depthImage, Matrix3f K){
    this->K=K;
    this->depthImage=depthImage;

    ushort depthPixel;
    Eigen::Vector3f worldPoint;
    cloudData pointElement;

    for(int i = 0;i<depthImage.cols;i++){
        for(int j=0;j<depthImage.rows;j++){
            ushort depthPixel = depthImage.at<ushort>(j,i);
            if(depthPixel<=1000) continue;
            worldPoint <<i*depthPixel,j*depthPixel,depthPixel;
            worldPoint = K.inverse()*worldPoint;
            pointElement.x=worldPoint[0];
            pointElement.y=worldPoint[1];
            pointElement.z=worldPoint[2];
            this->cloud.push_back(pointElement);
        }
    }
    //std::cout<<"cloud data has "<<this->cloud.size()<<" points"<<std::endl;
}


pointCloud::pointCloud(pcl::PointCloud<pcl::PointXYZ>* pcl){
    cloudData pointElement;
    for(int i=0;i<pcl->size();i++){
        pcl::PointXYZ p = pcl->at(i);
        pointElement.x=p.getArray3fMap()[0];
        pointElement.y=p.getArray3fMap()[1];
        pointElement.z=p.getArray3fMap()[2];
        this->cloud.push_back(pointElement);
    }
    //std::cout << "cloud has "<<this->cloud.size()<<" points"<<std::endl;
}

void pointCloud::T( Eigen::Isometry3f t){

    for(int i =0;i<this->cloud.size();i++){
        cloudData c = cloud.at(i);
        Eigen::Vector3f v;
        v<<c.x,c.y,c.z;
        v=t*v;
        cloud.at(i).x=v[0];
        cloud.at(i).y=v[1];
        cloud.at(i).z=v[2];
    }
}

pointCloud::~pointCloud(){
    this->cloud.clear();
    this->depthImage.release();
    this->pclCloud()->clear();
}

pointCloud::pointCloud(){

}


pcl::PointCloud<pcl::PointXYZ>* pointCloud::pclCloud(){
    pcl::PointCloud<pcl::PointXYZ>* out = new pcl::PointCloud<pcl::PointXYZ>();
    for(int i=0;i<cloud.size();i++){
        cloudData c = cloud.at(i);
        out->push_back(pcl::PointXYZ(c.x,c.y,c.z));
    }
    return out;
}

