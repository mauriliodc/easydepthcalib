#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dirent.h>
#include <getopt.h>
#include "calibrationMatrix.h"
//=============================================================================
int pgmfilter(const struct dirent *dir)
// post: returns 1/true if name of dir ends in .mp3
{
    const char *s = dir->d_name;
    int len = strlen(s) - 4;    // index of start of . in .mp3
    if(len >= 0)
    {
        if (strncmp(s + len, ".pgm", 4) == 0)
        {
            return 1;
        }
    }
    return 0;
}

static int one (const struct dirent *unused)
{
    return 1;
}
//=============================================================================
string inputdir="";
string outputdir="out";
string calibfilename="calibration.mal";

int main(int argc, char **argv)
{
    int c;
    if(argc<=1){
        printf("This node is intended to use to calibrate dumped data offline\n");
        printf("You need:\n- an input directory with the distorted depth images\n- an output directory where the undistorted images will be saved\n- the calibration file for the sensor used\n");
        printf("Usage:\n");
        printf("--input  -i string inputdirectory\n");
        printf("--output -o string outputdirectory\n");
        printf("--calib  -c string calibration file\n");
        printf("Example:\n");
        printf("./batchCalibration -i distortedDirectory -o undistortedDirectory -c depthcamera_calibration_file.mal");
        exit(1);
    }
    while (1)
    {
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"input",        required_argument,       0, 'i'},
            {"output",       required_argument,       0, 'o'},
            {"calib",       required_argument,        0, 'c'},
            {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long (argc, argv, "i:o:c:",long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            /* If this option set a flag, do nothing else now. */
            if (long_options[option_index].flag != 0)
                break;
            printf ("option %s", long_options[option_index].name);
            if (optarg)
                printf (" with arg %s", optarg);
            printf ("\n");
            break;

        case 'i':
            printf ("-\tinput dir `%s'\n", optarg);
            inputdir=optarg;
            break;

        case 'o':
            printf ("-\toutput dir `%s'\n", optarg);
            outputdir=optarg;
            break;

        case 'c':
            printf ("-\tcalibration file `%s'\n", optarg);
            calibfilename=optarg;
            break;
        default:
            abort ();
        }
    }

    std::cout<< calibfilename<<std::endl;
    calibrationMatrix multiplier((char*)calibfilename.c_str());
    struct dirent **eps;
    int n;
    string fullPath="./";
    fullPath.append(inputdir);
    fullPath.append("/");
    n = scandir (fullPath.c_str(), &eps, pgmfilter, alphasort);
    if (n >= 0)
    {
        int cnt;
        for (cnt = 0; cnt < n; ++cnt){
            std::cout<< "opening "<<eps[cnt]->d_name<<std::endl;
            cv::Mat image;
            std::string filename="./";
            filename.append(inputdir);
            filename.append("/");
            filename.append(eps[cnt]->d_name);
            std::cout<<"OPENING:\t "<<filename<<std::endl;
            image = cv::imread(filename,CV_LOAD_IMAGE_UNCHANGED);


            int cols=image.cols;
            int rows=image.rows;
            cv::Point p;
            ushort v;
            for (int i=0;i<cols;i++){
                for(int j=0;j<rows;j++){
                    p.x=i;
                    p.y=j;
                    v=image.at<ushort>(p);
                    if(v!=0){
                        v=multiplier.cell(p.y,p.x,v)*v;
                    } else{
//                        std::cout<<"[v: "<<v<<std::endl;
//                        std::cout<<"m: "<<multiplier.cell(p.y,p.x,v)<<std::endl;
                    }
                    //                    if(multiplier.cell(p.y,p.x,v/10)!=1.0f)
                    //                        std::cout << " is "<<v<<std::endl;
                    image.at<ushort>(p)=(ushort)v;
                }
            }
            std::string outDir=("./");
            outDir.append(outputdir);
            outDir.append("/");
            outDir.append(eps[cnt]->d_name);
            cv::imwrite(outDir,image);
            std::cout<< "saved "<<outDir<<std::endl;
        }
    }
    else
        perror ("Couldn't open the directory");


    return 0;
}
