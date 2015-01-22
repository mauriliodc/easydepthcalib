#include "randomPointsActor.h"

randomPointsActor::randomPointsActor(){

}

randomPointsActor::~randomPointsActor(){

}

void randomPointsActor::draw(){
    glBegin(GL_POINTS); //starts drawing of points
    glVertex3f(1.0f,1.0f,0.0f);//upper-right corner
    glVertex3f(-1.0f,-1.0f,0.0f);//lower-left corner
    glEnd();//end drawing of points
}
