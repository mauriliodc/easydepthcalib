#ifndef LC_H
#define LC_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "pointCloud.h"
struct laserCartesian{
    float x;
    float y;
};


class laserData{
public:
    laserData(std::ifstream &inputData,int meters=1);
    ~laserData();
    static pointCloud* convertToPointCloud(laserData* d,int slices, float noiseRatio=0);
    std::vector<laserCartesian> data;
};

#endif
