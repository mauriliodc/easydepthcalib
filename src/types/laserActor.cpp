#include "laserActor.h"

LaserActor::LaserActor(laserData* l)
{
    this->l=l;

}

LaserActor::~LaserActor(){

}

void LaserActor::draw(){
    glColor3f(this->color.r, this->color.g, this->color.b);
    glBegin(GL_POINTS);
    Eigen::Vector3f v;
    for(int i=0;i<this->l->data.size();i++){
        struct laserCartesian c;
        c=this->l->data.at(i);
        v<<c.x,c.y,0;
        v=transform*v;
        glVertex3f(v[0],v[1],v[2]);
    }
    glEnd();
    
}
