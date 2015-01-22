#ifndef LACTR_H
#define LACTR_H

#include "actor.h"
#include "laserData.h"
class LaserActor : public actor {
public:
    LaserActor(laserData* l);
    ~LaserActor();

    void draw();
    laserData* l;

    struct _color{
        float r;
        float g;
        float b;
    } color;

};

#endif
