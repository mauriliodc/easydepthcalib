#ifndef PC_H
#define PC_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "pointCloud.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
using namespace Eigen;
using namespace std;

class planeCloud : public pointCloud{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    planeCloud();
    ~planeCloud();

    Vector4f model;
    Vector3f com;
    //static void extractPlaneCloud(planeCloud* plane,pointCloud* p);
    static void extractPlanesCloud(std::vector<planeCloud*>* planes,pointCloud* p,int distanceThreshold=15, int residualPointsThreshold=20000, int inliersThreshold=20000 );
    static void dataAssociation(std::vector<planeCloud*>* planeList1,std::vector<planeCloud*>* planeList2,std::vector<std::pair<int,int> >* assoc);
};

#endif
