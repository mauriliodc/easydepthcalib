#include "calibration.h"

void calibration::voxelize(pcl::PointCloud<pcl::PointXYZ>* inCloud,pcl::PointCloud<pcl::PointXYZ>* outCloud, float voxelLeaf){
    //std::cout<<"IN has "<<inCloud->size();
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (inCloud->makeShared());
    float _voxelLeaf =voxelLeaf;
    sor.setLeafSize ((float)_voxelLeaf, (float)_voxelLeaf, (float)_voxelLeaf);
    sor.filter (*outCloud);
    //std::cout<<" OUT has "<<outCloud->size()<<std::endl;
}
void calibration::computeCenterSquareCloud(cv::Mat& depthImage,Eigen::Matrix3f k, pcl::PointCloud<pcl::PointXYZ>* outCloud, int width, int height, float c0, float c1, float c2, float c3){
    outCloud->clear();
    int cols=depthImage.cols;
    int rows=depthImage.rows;
    for(int i=cols/2-width;i<cols/2+width;i++){
        for(int j=rows/2-height;j<rows/2+height;j++){
            ushort depthPixel = depthImage.at<ushort>(j,i);
            Eigen::Vector3f worldPoint;
            worldPoint <<i*depthPixel,j*depthPixel,depthPixel;
            worldPoint = k.inverse()*worldPoint;
            outCloud->push_back(pcl::PointXYZ(worldPoint[0],worldPoint[1],worldPoint[2]));
        }
    }
}
void calibration::computerCenterPlane(pcl::PointCloud<pcl::PointXYZ>* inCloud,Eigen::Vector4f& model, int distanceThreshold){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (inCloud->makeShared ());
    seg.segment (*inliers, *coefficients);

    model[0]=coefficients->values[0];
    model[1]=coefficients->values[1];
    model[2]=coefficients->values[2];
    model[3]=coefficients->values[3];
}

void calibration::computeNormals(pcl::PointCloud<pcl::PointXYZ>* inCloud, pcl::PointCloud<pcl::Normal>* normals, float searchRadius){
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr p(inCloud);
    //todo: check here
    ne.setInputCloud (p);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (searchRadius);
    ne.compute (*normals);
}

void calibration::pointrejection(Eigen::Vector3f* referenceNormal,
                                 float rejectionValue,
                                 pcl::PointCloud<pcl::PointXYZ>* inCloud,
                                 pcl::PointCloud<pcl::Normal>* normals,
                                 pcl::PointCloud<pcl::PointXYZ>* outCloud,
                                 std::vector<bool>* validIndeces ){
    pcl::Normal n;
    for(unsigned int i=0; i<inCloud->size();i++){
        n=normals->at(i);
        Eigen::Vector3f n1(n.normal_x,n.normal_y,n.normal_z);
        Eigen::Vector3f cross=n1.cross(*referenceNormal);
        if(cross.norm()>rejectionValue){
            validIndeces->push_back(false);
            outCloud->push_back(pcl::PointXYZ(inCloud->at(i)));
        }
        else{
            validIndeces->push_back(true);
        }

    }
}
void calibration::computeErrorPerPoint(pcl::PointCloud<pcl::PointXYZ>* inCloud,
                                       pcl::PointCloud<pcl::PointXYZ>* errorCloud,
                                       Eigen::Vector3f planeOrigin,
                                       Eigen::Vector3f planeCoefficient,
                                       std::vector<bool>* validIndeces ){
    pcl::PointXYZ p;
    pcl::PointXYZ pp;
    Eigen::Vector3f res;
    for(unsigned int i=0; i<inCloud->size();i++){
        p=inCloud->at(i);
        pcl::geometry::project( Eigen::Vector3f(p.x,p.y,p.z),
                                planeOrigin,
                                planeCoefficient,
                                res);
        pp.x=res(0);
        pp.y=res(1);
        pp.z=res(2);
        if(validIndeces->at(i)==false){
            pp.x=p.x;
            pp.y=p.y;
            pp.z=p.z;
        }
        errorCloud->push_back(pp);

    }
}
void calibration::computeCalibrationMatrix(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                           pcl::PointCloud<pcl::PointXYZ> &errorCloud,
                                           Eigen::Matrix3f k,
                                           std::vector<bool>* validIndeces ,
                                           calibrationMatrix &cM){
    pcl::PointXYZ point;
    pcl::PointXYZ projected_point;
    for(unsigned int i=0; i<inCloud.size();i++){
        point=inCloud.at(i);
        projected_point=errorCloud.at(i);

        Eigen::Vector3f diff = point.getVector3fMap() - projected_point.getVector3fMap();
        Eigen::Vector3f e=projected_point.getVector3fMap();
        float measuredDistance = diff.norm();
        Eigen::Vector3f localPoint;
        localPoint = k*Eigen::Vector3f(point.getArray3fMap());
        //PIXEL COORDINATES (DEPTH IMAGE)
        localPoint[0]/=localPoint[2];
        localPoint[1]/=localPoint[2];

        //std::cout<<"MAX= "<<cM.maxDepth<<" CALIB: "<<localPoint.y()<<" "<<localPoint.x()<<" "<<localPoint.z()<<std::endl;
        if(localPoint.z()>=cM.maxDepth){
            std::cout<<" [X] "<<std::endl;
            return;
        }
        if(point.z<=projected_point.z){
            cM.cell(localPoint.y(),
                    localPoint.x(),
                    localPoint.z(),
                    (measuredDistance+localPoint.z())/localPoint.z());

        }
        if(point.z>projected_point.z){
            cM.cell(localPoint.y(),
                    localPoint.x(),
                    localPoint.z(),
                    (localPoint.z()-measuredDistance)/localPoint.z());

        }

        cM.increment(localPoint.y(),localPoint.x(),localPoint.z());

    }

}
void calibration::calibratePointCloudWithMultipliers(pcl::PointCloud<pcl::PointXYZ> &inCloud,
                                                     pcl::PointCloud<pcl::PointXYZ> &outCloud,
                                                     calibrationMatrix &cM,
                                                     Eigen::Matrix3f k){
    pcl::PointXYZ point;
    for(unsigned int i=0; i<inCloud.size();i++){

        point=inCloud.at(i);
        Eigen::Vector3f localPoint;
        localPoint = k*Eigen::Vector3f(point.getArray3fMap());
        localPoint[0]/=localPoint[2];
        localPoint[1]/=localPoint[2];
        localPoint[2]*=cM.cell(localPoint[1],localPoint[0],localPoint[2]);

        localPoint[0]*=localPoint[2];
        localPoint[1]*=localPoint[2];
        Eigen::Vector3f newLocal;

        newLocal=k.inverse()*localPoint;
        outCloud.push_back(pcl::PointXYZ(newLocal[0],newLocal[1],newLocal[2]));
    }





}
