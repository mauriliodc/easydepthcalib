#include "pointCloudActor.h"

PointCloudActor::PointCloudActor(pointCloud p, float r, float g, float b){
    this->p=p;
    this->color.r=r;
    this->color.g=g;
    this->color.b=b;
}

PointCloudActor::~PointCloudActor(){

}

void PointCloudActor::draw(){
    glBegin(GL_POINTS);
    glColor3f(this->color.r,this->color.g,this->color.b);
    struct cloudData c;
    Eigen::Vector3f v;
    for(int i = 0;i<this->p.cloud.size();i++){
        c=p.cloud.at(i);
        v<<c.x,c.y,c.z;
        v=transform*v;
        glVertex3f(v[0],v[1],v[2]);
    }
    glColor3f(1.0f,1.0f,1.0f);
    glEnd();
}

void PointCloudActor::feedPointCloud(pointCloud p){
    this->p=p;
}
