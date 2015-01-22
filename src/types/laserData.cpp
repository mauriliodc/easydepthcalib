#include "laserData.h"

laserData::laserData(std::ifstream &inputData,int meters){
    std::string chunk;
    int numberOfPoints;
    inputData>>chunk;
    std::cout<<chunk<<" ";
    inputData>>chunk;
    std::cout<<chunk<<" POINTS: ";
    inputData>>numberOfPoints;
    std::cout<<numberOfPoints<<std::endl;
    for(int i =0;i<numberOfPoints;i++){
        laserCartesian c;
        inputData>>c.x;
        c.x*=meters;
        inputData>>c.y;
        c.y*=meters;
        data.push_back(c);
    }

    std::cout<< "added "<<data.size()<<" points"<<std::endl;

}




laserData::~laserData(){
    data.clear();
}


pointCloud* laserData::convertToPointCloud(laserData* d,int slices, float noiseRatio){
    pointCloud* p= new pointCloud();
    struct cloudData c;
    for(int i=0;i<d->data.size();i++){
        for(int j=0;j<slices;j++){
            c.x=d->data.at(i).x+((float)rand()/RAND_MAX)*noiseRatio;
            c.y=d->data.at(i).y+((float)rand()/RAND_MAX)*noiseRatio;
            c.z=j*10+((float)rand()/RAND_MAX)*noiseRatio*0;
            p->cloud.push_back(c);

        }
    }
    std::cout<< "cloud now has "<<p->cloud.size()<<" points"<<std::endl;
    return p;
}
