#ifndef CALIBM_H
#define CALIBM_H

#include <map>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>

typedef float innertype;

using namespace std;

class calibrationMatrix{
public:
    calibrationMatrix(int rows, int cols, int maxDepth, int tileSize, int depthRes);
    calibrationMatrix(char* filename);
    //~calibrationMatrix();
    innertype cell(int r, int c, int d);
    void cell(int r, int c, int d, innertype mply);
    void increment(int r, int c, int d);
    void serialize(char* filename);
    void serializeNN(char* filename);
    void deserialize(char* filename);
    calibrationMatrix* downsample(int dxy, int dd);
    void clear();
    void worldToMap(int& ir, int& ic, int& id, int r, int c, int d);
    void mapToWorld(int& r, int& c, int& d,int ir, int ic, int id);
    void dumpSensorImages();
    void dumpCovariance();
    void syncToFloat();
    void dumpMe();
    innertype  growTop(int row, int col, int dep);
    innertype  growBottom(int row, int col, int dep);
    innertype  growLeft(int row, int col, int dep);
    innertype  growRight(int row, int col, int dep);

    void getStats();
    innertype cellNN(int r, int c, int d);
    innertype getFloat(int r, int c, int d);
    int maxDepth;
    int tileSize;
    int tilePow;
    int depthRes;
    int depthPow;
    int rows;
    int cols;
    int layers;
    bool useKernel;

    innertype*** _data;
    innertype*** _staticData;
    innertype*** _hits;
    innertype*** _covariance;

};


#endif
