#ifndef PCA_H
#define PCA_H

#include <actor.h>
#include <cstdlib>
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
#include "pointCloud.h"

using namespace Eigen;
using namespace std;

class PointCloudActor : public actor{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudActor(pointCloud p, float r=1, float g=1, float b=1);
    inline PointCloudActor(){}
    void feedPointCloud(pointCloud p);

    ~PointCloudActor();

    void draw();
    pointCloud p;
    struct _color{
        float r;
        float g;
        float b;
    } color;
};

#endif
