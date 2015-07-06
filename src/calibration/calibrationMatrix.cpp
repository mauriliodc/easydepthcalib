#include "calibrationMatrix.h"

calibrationMatrix::calibrationMatrix(int rows, int cols, int maxDepth, int tileSize, int depthRes){
    useKernel=false;
    this->maxDepth=maxDepth;
    this->tileSize=tileSize;
    this->depthRes=depthRes;
    this->rows=rows/tileSize;
    this->cols=cols/tileSize;
    this->layers=this->maxDepth/this->depthRes;



    cerr << "tailsaiz: " << this->tileSize  << endl;
    cerr << "allocating LAYERS: " << layers << " ROWS: " << this->rows << " COLS: " << this->cols << " cells" << endl;
    _data = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _data[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _data[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _data[i][j][k]=1.0f;

            }
        }
    }

    _staticData = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _staticData[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _staticData[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=1.0f;

            }
        }
    }


    _hits = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _hits[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _hits[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _hits[i][j][k]=1.0f;

            }
        }
    }


    _covariance = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _covariance[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _covariance[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _covariance[i][j][k]=1.0f;

            }
        }
    }


}


void calibrationMatrix::clear(){
    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            delete(_data[i][j]);
        }
        delete(_data[i]);
    }
    delete(_data);


    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            delete(_staticData[i][j]);
        }
        delete(_staticData[i]);
    }
    delete(_staticData);


    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            delete(_hits[i][j]);
        }
        delete(_hits[i]);
    }
    delete(_hits);


    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            delete(_covariance[i][j]);
        }
        delete(_covariance[i]);
    }
    delete(_covariance);

}

innertype calibrationMatrix::cell(int r, int c, int d){
    //if(d>this->layers*this->depthRes){
    if(d>this->maxDepth){
        printf("[WARNING] requested depth (%d) out of calib range\n",d);
        return 1;
    }
    return ((_data[d/depthRes][r/tileSize][c/tileSize])/(_hits[d/depthRes][r/tileSize][c/tileSize]));
}

void calibrationMatrix::cell(int r, int c, int d, innertype mply){
//    std::cout<<"cell "<<d<<" "<<r<<" "<<c<<std::endl;
    for(int i=-2;i<3;i++){
        for(int j=-2;j<3;j++){
            for(int k=-2;k<3;k++){
                if(     (d/depthRes+i)>=0 &&
                        (r/tileSize+j)>=0 &&
                        (c/tileSize+k)>=0 &&
                        (d/depthRes+i)<layers &&
                        (r/tileSize+j)<rows &&
                        (c/tileSize+k)<cols){
                    _data[d/depthRes+i][r/tileSize+j][c/tileSize+k]+=mply;//*(1-abs(j));
                    _hits[d/depthRes+i][r/tileSize+j][c/tileSize+k]+=1.0f;
                    //std::cout<<"\t\tGOOD "<<_data[d/depthRes+i][r/tileSize+j][c/tileSize+k]<<" "<< _hits[d/depthRes+i][r/tileSize+j][c/tileSize+k]<<" "<<_data[d/depthRes+i][r/tileSize+j][c/tileSize+k]/_hits[d/depthRes+i][r/tileSize+j][c/tileSize+k]<<  std::endl;
                }
                else{
//                    std::cout<<"\tfail @ "<< (d/depthRes+i)<<" (maxdepth:"<<layers<<") "<<
//                                           (r/tileSize+j)<<" (rows: "<<rows<<") "<<
//                                           (c/tileSize+k)<<" (cols: "<<cols<<") "<<std::endl;
                }
                //_covariance[(d+i)/depthRes][(r+j)/tileSize][(c+k)/tileSize]+=mply*mply;
            }
        }
    }


}

void calibrationMatrix::syncToFloat(){

    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=_data[i][j][k]/_hits[i][j][k];
            }
        }
    }
}

innertype calibrationMatrix::getFloat(int r, int c, int d){
    return _staticData[d/depthRes][r/tileSize][c/tileSize];
}

void calibrationMatrix::increment(int r, int c, int d){
//    for(int i=-2;i<3;i++){
//        for(int j=-2;j<3;j++){
//            for(int k=-2;k<3;k++){
//                if(     (d/depthRes+i)>=0 &&
//                        (r/tileSize+j)>=0 &&
//                        (c/tileSize+k)>=0 &&
//                        (d/depthRes+i)<layers &&
//                        (r/tileSize+j)<rows &&
//                        (c/tileSize+k)<cols){
//                    _hits[d/depthRes+i][r/tileSize+j][c/tileSize+k]+=1.0f;
//                    //std::cout<<"write at @ "<<(d/depthRes+i)<<" (maxdepth:"<<maxDepth<<") "<<(r/tileSize+j)<<" (rows: "<<rows<<") "<<(c/tileSize+k)<<" (cols: "<<cols<<") "<<std::endl;
//                }

//            }
//        }
//    }

}

void calibrationMatrix::getStats(){
    int unos=0;
    for (int i=0; i<layers; i++){

        for (int j=0; j< rows; j++){

            for (int k=0; k< cols; k++){

                if(_hits[i][j][k]==1){
                    unos++;
                }

            }
        }
    }
    std::cout << "############ missing data on "<<unos<<"/"<<layers*rows*cols<<" " <<(innertype)((innertype)unos/((innertype)(layers*rows*cols))) <<std::endl;
}

innertype  calibrationMatrix::growLeft(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && row<(rows-1))
    {
        growLeft(++row,col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}

innertype  calibrationMatrix::growRight(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && row>0)
    {
        growLeft(--row,col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}


innertype  calibrationMatrix::growTop(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && col>0)
    {
        growLeft(row,--col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}


innertype  calibrationMatrix::growBottom(int row, int col, int dep){
    if(_hits[dep][row][col]==1.0f && col<(cols-1))
    {
        growLeft(row,++col,dep);
    }
    return _data[dep][row][col]/_hits[dep][row][col];
}

innertype calibrationMatrix::cellNN(int row, int col, int dep){
    innertype v=0;
    v+=growLeft(row,col,dep);
    v+=growRight(row,col,dep);
    v+=growTop(row,col,dep);
    v+=growBottom(row,col,dep);
    return v/4;
}



void calibrationMatrix::worldToMap(int& ir, int& ic, int& id, int r, int c, int d){
    ir=r/tileSize;
    ic=c/tileSize;
    id=d/depthRes;
}

void calibrationMatrix::mapToWorld(int& r, int& c, int& d,int ir, int ic, int id){
    r=ir/tileSize;
    c=ic/tileSize;
    d=id/depthRes;
}





//-----------------------------------------------------------------------------------SAVE TO FILE
void calibrationMatrix::serialize(char* filename){
    std::cout<<"opening handle...";
    std::ofstream writer(filename);
    std::cout<<"saving...";
    std::cout<< layers << " "<<rows << " "<<cols<<" "<<depthRes<<std::endl;
    std::cout.flush();
    writer<<layers<<" "<<rows<<" "<<cols<<" "<<" "<<depthRes<< std::endl;

    for (int i=0; i<this->layers; i++){
        for (int j=0; j< this->rows; j++){
            for (int k=0; k< this->cols; k++){
                writer<<_data[i][j][k]<< " " << _hits[i][j][k]<<" " << sqrt(((_data[i][j][k]*_data[i][j][k])/_hits[i][j][k])-(_data[i][j][k]/_hits[i][j][k])*(_data[i][j][k]/_hits[i][j][k]))<<" ";
            }
        }
        writer<<std::endl;
    }
    writer.close();
    std::cout<<"done"<<std::endl;

}

void calibrationMatrix::serializeNN(char* filename){
    std::cout<<"opening handle...";
    std::ofstream writer(filename);
    std::cout<<"saving...";
    std::cout<< layers << " "<<rows << " "<<cols<<" "<<depthRes<<std::endl;
    std::cout.flush();
    writer<<layers<<" "<<rows<<" "<<cols<<" "<<" "<<depthRes<< std::endl;
    innertype _d=0;
    for (int i=0; i<this->layers; i++){
        for (int j=0; j< this->rows; j++){
            for (int k=0; k< this->cols; k++){
                _d =_data[i][j][k];
                if(_hits[i][j][k]==1)_d=this->cellNN(j,k,i);
                writer<<_d<<" "<< _hits[i][j][k]<<" " << sqrt(((_data[i][j][k]*_data[i][j][k])/_hits[i][j][k])-(_data[i][j][k]/_hits[i][j][k])*(_data[i][j][k]/_hits[i][j][k]))<<" ";
            }
        }
        writer<<std::endl;
    }
    writer.close();
    std::cout<<"done"<<std::endl;

}
//-----------------------------------------------------------------------------------LOAD FROM FILE

calibrationMatrix::calibrationMatrix(char* filename){
    std::ifstream myfile (filename);
    int layers;
    int rows;
    int cols;
    int depthres;

    myfile >> layers >> rows >> cols >> depthres;
    std::cout << "allocating from file "<<layers<<" " <<rows<<" "<<cols <<" "<<depthres << std::endl;
    std::cout.flush();

    this->maxDepth=depthres*layers;
    this->tileSize=640/cols;
    this->depthRes=depthres;
    this->rows=rows;
    this->cols=cols;
    this->layers=layers;

    _data = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _data[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _data[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _data[i][j][k]=1.0f;

            }
        }
    }

    _staticData = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _staticData[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _staticData[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _staticData[i][j][k]=1.0f;

            }
        }
    }


    _hits = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _hits[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _hits[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _hits[i][j][k]=1.0f;

            }
        }
    }


    _covariance = new innertype**[layers];
    for (int i=0; i<layers; i++){
        _covariance[i] = new innertype* [rows];
        for (int j=0; j< rows; j++){
            _covariance[i][j]=new innertype[cols];
            for (int k=0; k< cols; k++){
                _covariance[i][j][k]=1.0f;

            }
        }
    }


    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                myfile>>_data[i][j][k];
                myfile>>_hits[i][j][k];
                myfile>>_covariance[i][j][k];
            }
        }
    }
}

void calibrationMatrix::deserialize(char* filename){
    std::ifstream myfile (filename);
    int layers;
    int rows;
    int cols;
    int buff;
    myfile >> layers >> rows >> cols >>buff;
    std::cout << " from file "<<layers<<" " <<rows<<" "<<cols << std::endl;
    std::cout.flush();

    if(layers!= this->layers && rows != this->rows && cols!=this->cols){
        std::cout << "file data incosistent"<<std::endl;
        std::cout << "should be "<<layers << " "<<rows <<" " << cols<<std::endl;
    }
    else{
        for (int i=0; i<layers; i++){

            for (int j=0; j< rows; j++){

                for (int k=0; k< cols; k++){
                    myfile>>_data[i][j][k];
                    myfile>>_hits[i][j][k];
                    myfile>>_covariance[i][j][k];
                }

            }

        }
    }
    myfile.close();
}

calibrationMatrix* calibrationMatrix::downsample(int dxy, int dd){
    calibrationMatrix* downsampled = new calibrationMatrix(rows*tileSize, cols*tileSize, maxDepth+depthRes*dd, dxy*tileSize, depthRes*dd);
    std::cerr << "allocated" << std::endl;
    std::cerr << "clearing" << std::endl;
    for (int i=0; i<downsampled->layers; i++){
        for (int j=0; j< downsampled->rows; j++){
            for (int k=0; k< downsampled->cols; k++){
                downsampled->_data[i][j][k] = 0;
                downsampled->_hits[i][j][k] = 0;
                downsampled->_covariance[i][j][k] = 0;
            }
        }
    }

    std::cerr << "filling" << std::endl;
    for (int i=0; i<layers; i++){
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                downsampled->_data[i/dd][j/dxy][k/dxy] += _data[i][j][k];
                downsampled->_hits[i/dd][j/dxy][k/dxy] += _hits[i][j][k];
                downsampled->_covariance[i/dd][j/dxy][k/dxy] = _covariance[i][j][k];
            }
        }
    }
    return downsampled;
}

void calibrationMatrix::dumpSensorImages(){
    for (int i=0; i<layers; i++){
        cv::Mat errorImage(rows,cols,CV_32FC1);
        cv::Mat error(rows,cols,CV_8UC1);
        errorImage=cv::Mat::zeros(rows,cols,CV_32FC1);
        cv::Point p;
        for (int j=0; j< rows; j++){
            for (int k=0; k< cols; k++){
                p.y=j;
                p.x=k;
                innertype v= _data[i][j][k]/_hits[i][j][k];
                if(_hits[i][j][k]==1)v=0;
                errorImage.at<innertype>(p)=(v-1)*3000+127;
            }
        }
        cv::flip(errorImage,errorImage,0);
        double min;
        double max;
        errorImage.convertTo(error,CV_8UC1);
        cv::minMaxIdx(error,&min,&max);
        std::cout <<"["<<i<<"]"<< " m: "<<min << " M: "<<max<<std::endl;
        cv::Mat dest;
        char filename[50];
        for(int colormap =0;colormap<1;colormap++){
            cv::applyColorMap(error,dest,colormap);
            sprintf(filename,"layer_%d.pgm",i);
            cv::imwrite(filename,dest);
        }

    }
}


void calibrationMatrix::dumpMe(){
    std::ifstream myfile ("layer_32_4096_ANN.txt");
    innertype m;
    cv::Mat ANN(480,640,CV_32FC1);
    cv::Point p;
    for (int i=0; i<480; i++){
        for (int j=0; j<640; j++){
            int buco;
            myfile>>buco;myfile>>buco;
            myfile>>m;
            p.x=j;
            p.y=i;
            ANN.at<innertype>(p)=(m-1)*3000+127;
        }
    }
    myfile.close();
    cv::flip(ANN,ANN,0);
    double min;
    double max;
    ANN.convertTo(ANN,CV_8UC1);
    cv::minMaxIdx(ANN,&min,&max);
    cv::Mat dest;
    for(int colormap =0;colormap<1;colormap++){
        cv::applyColorMap(ANN,dest,colormap);
        cv::imwrite("ANN_OUT.PGM",dest);
    }
}




void calibrationMatrix::dumpCovariance(){
    for (int i=0; i<layers; i++){
        cv::Mat errorImage(rows,cols,CV_32FC1);
        cv::Mat error(rows,cols,CV_8UC1);
        errorImage=cv::Mat::zeros(rows,cols,CV_32FC1);
        cv::Point p;
        for (int j=0; j< rows; j++){

            for (int k=0; k< cols; k++){
                p.y=j;
                p.x=k;
                innertype v= sqrt(((_data[i][j][k]*_data[i][j][k])/_hits[i][j][k])-(_data[i][j][k]/_hits[i][j][k])*(_data[i][j][k]/_hits[i][j][k]));
                errorImage.at<innertype>(p)=v;//(v-1)*3000+127;

            }

        }
        cv::flip(errorImage,errorImage,0);
        double min;
        double max;
        cv::minMaxIdx(errorImage,&min,&max);
        std::cout << "m: "<<min << " M: "<<max<<std::endl;
        errorImage.convertTo(error,CV_8UC1, 255.0/(max - min), -min * 255.0/(max - min));
        cv::Mat dest;
        char filename[500];
        for(int colormap =0;colormap<1;colormap++){
            cv::applyColorMap(error,dest,colormap);
            sprintf(filename,"covariances_%d.pgm",i);
            cv::imwrite(filename,dest);


        }

    }
}





