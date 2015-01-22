#include "planeCloudActor.h"
#define planeSize 3000.0f
PlaneCloudACtor::PlaneCloudACtor(planeCloud& p, float r, float g, float b, float a){
    this->p=&p;
    this->color.r=r;
    this->color.g=g;
    this->color.b=b;
    this->color.a=a;
}

PlaneCloudACtor::~PlaneCloudACtor(){

}

float PlaneCloudACtor::azimuth(Vector3f v) {
    return atan2(v(1),v(0));
}

float PlaneCloudACtor::elevation(Vector3f v) {
    return atan2(v(2), v.head<2>().norm());
}


void PlaneCloudACtor::draw(){
    if(this->hasToDraw==false) return;

    float d=this->p->model[3];
    float azimuth=PlaneCloudACtor::azimuth(this->p->model.head(3));
    float elevation=PlaneCloudACtor::elevation(this->p->model.head(3));


    glColor3f(1.0f,0.0f,0.0f);
    glLineWidth(10.f);
    glBegin(GL_LINES);
    glVertex3f(this->p->com[0],this->p->com[1],this->p->com[2]);
    glVertex3f(this->p->com[0]+this->p->model[0]*100,
            this->p->com[1]+this->p->model[1]*100,
            this->p->com[2]+this->p->model[2]*100);
    glEnd();



    glColor4f(this->color.r, this->color.g, this->color.b, this->color.a);

    glPushMatrix();
    glRotatef(RAD2DEG(azimuth),0.,0.,1.);
    glRotatef(RAD2DEG(elevation),0.,-1.,0.);
    glTranslatef(-d,0.,0.);

    glBegin(GL_QUADS);
    glNormal3f(-1,0,0);
    glVertex3f(0,-planeSize, -planeSize);
    glVertex3f(0, planeSize, -planeSize);
    glVertex3f(0, planeSize,  planeSize);
    glVertex3f(0, -planeSize,  planeSize);
    glEnd();
    glPopMatrix();

}
