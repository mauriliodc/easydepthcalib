#include "planeCloud.h"

planeCloud::planeCloud(){

}

//distanceThreshold - distance in mm for a point to be part of a model
//residualPointsThreshold - number of point remaining in a cloud before stopping the iterations
//inliersThreshold - points in a plane to be considered a valid model
void planeCloud::extractPlanesCloud(std::vector<planeCloud*>* planes,pointCloud* p, int distanceThreshold, int residualPointsThreshold, int inliersThreshold){

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pclCloud.is_dense = true;
    pcl::PointXYZ pXYZ;
    struct cloudData d;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);


    for(int i =0;i<p->cloud.size();i++){
        pXYZ.x=p->cloud.at(i).x;
        pXYZ.y=p->cloud.at(i).y;
        pXYZ.z=p->cloud.at(i).z;
        pclCloud.push_back(pXYZ);
    }

    while(pclCloud.size()>residualPointsThreshold){
        seg.setInputCloud (pclCloud.makeShared());
        seg.segment (*inliers, *coefficients);
        if(inliers->indices.size()==0) return;
        if (inliers->indices.size() > inliersThreshold)
        {
            //std::cout<<"plane has "<<inliers->indices.size ()<< " points "<<std::endl;
            planeCloud* plane = new planeCloud();
            for (int i = 0; i < inliers->indices.size (); ++i){
                d.x=pclCloud.points[inliers->indices[i]].x;
                d.y=pclCloud.points[inliers->indices[i]].y;
                d.z=pclCloud.points[inliers->indices[i]].z;
                plane->cloud.push_back(d);
            }
            plane->model[0]=(coefficients->values[0]);
            plane->model[1]=(coefficients->values[1]);
            plane->model[2]=(coefficients->values[2]);
            plane->model[3]=(coefficients->values[3]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(pclCloud,inliers->indices,centroid);
            plane->com=centroid.head(3);
            //std::cout<<"COM "<<plane->com.transpose()<<std::endl;
            planes->push_back(plane);

        }
        else{
            //std::cout<<"plane has "<<inliers->indices.size ()<< " points, rejected "<<std::endl;
            //std::cout<<"points: "<<pclCloud.size()<<" ";
        }
        int old=pclCloud.size();
        pcl::ExtractIndices<pcl::PointXYZ> extract ;
        extract.setInputCloud (pclCloud.makeShared());
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter(pclCloud);
        inliers->indices.clear();
        //std::cout<<"now: "<<pclCloud.size()<<" "<<std::endl;
    }

}

void planeCloud::dataAssociation(std::vector<planeCloud*>* planeList1,std::vector<planeCloud*>* planeList2,std::vector<std::pair<int,int> >* assoc){
    std::vector<int> planeIndex1;
    std::vector<int> planeIndex2;

    std::vector< std::pair< std::pair<int,int>, float> > assocWithWeights;

    float wR=1.0f; //PESI
    float wT=1.0f; //PESI

    for(int i =0;i<planeList1->size();i++){
        planeIndex1.push_back(i);
    }

    for(int j =0;j<planeList2->size();j++){
        planeIndex2.push_back(j);
    }

    for(int i=0;i<planeIndex1.size();i++){
        planeCloud* plane1=planeList1->at(planeIndex1.at(i));
        float bestError=-1;
        std::pair<int, int> bestCouple;
        bestCouple=std::make_pair(-1,-1);

        for(int j=0;j<planeIndex2.size();j++){

            planeCloud* plane2=planeList2->at(planeIndex2.at(j));
            float ghibli=(plane1->model[0]-plane2->model[0])+(plane1->model[1]-plane2->model[1])+(plane1->model[2]-plane2->model[2]);
            ghibli=ghibli*ghibli;

            float w = (plane1->model[3]/1000*plane1->model[3]/1000)-(plane2->model[3]/1000*plane2->model[3]/1000);
            w=w*w;
            std::cout<<"Squared normal difference between plane ["<<i<<"] and ["<<j<<"] r = "<<ghibli;
            std::cout<<" w ="<<w<<std::endl;
            float error=ghibli+w;
            if(bestError==-1){
                bestError=error;
                bestCouple=std::make_pair(i,j);
            }
            else if(bestError>error){
                bestError=error;
                bestCouple=std::make_pair(i,j);
            }
        }
        assoc->push_back(bestCouple);
    }


}


planeCloud::~planeCloud(){

}
