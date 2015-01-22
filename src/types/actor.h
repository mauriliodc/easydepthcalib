#ifndef ACTOR_H
#define ACTOR_H

#include <GL/freeglut.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class actor {
public:
    inline actor(){

        this->name=-1;
        this->transform=Eigen::Matrix4f::Identity();
    }
    virtual ~actor(){}
    virtual void draw()=0;
    bool hasToDraw;
    int name;
    inline void setDraw(bool state){
        this->hasToDraw=state;
    }

    Eigen::Isometry3f transform;

};

#endif
