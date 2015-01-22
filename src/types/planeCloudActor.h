#ifndef PLCA_H
#define PLCA_H

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
#include "planeCloud.h"

using namespace Eigen;
using namespace std;

class PlaneCloudACtor : public actor{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PlaneCloudACtor(planeCloud& p, float r=1, float g=1, float b=1, float a=0.2f);
    ~PlaneCloudACtor();

    static float azimuth(Vector3f v);
    static float elevation(Vector3f v);

    void draw();
    planeCloud* p;
    struct _color{
        float r;
        float g;
        float b;
        float a;
    } color;
};

#endif
