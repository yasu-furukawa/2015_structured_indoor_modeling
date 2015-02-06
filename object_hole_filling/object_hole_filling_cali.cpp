#include "SLIC.h"
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <string>
#include <gflags/gflags.h>
#include "../base/file_io.h"
#include "../base/point_cloud.h"
#include "../base/panorama.h"
#include <vector>
#include <typeinfo>
#include "object_hole_filling.h"
#include "depth_filling.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

DEFINE_string(config_path,"lumber.configuration","Path to the configuration file");
DEFINE_int32(label_num,3000,"Number of superpixel");

int main(int argc, char **argv){

  gflags::ParseCommandLineFlags(&argc,&argv,true);
  if(! (FLAGS_config_path.length() > 0)){
    cout<<"Usage: Object_hole_filling /path to your configuration file"<<endl;
  }
  //get path to data
  char pathtodata[100];
  int startid, endid;
  ifstream confin(FLAGS_config_path.c_str());
  confin.getline(pathtodata,100);
  confin>>startid>>endid;
  confin.close();
  string pathtodata_s(pathtodata);
  FileIO file_io(pathtodata_s);
  startid = 0;
  for (int id=startid; id<startid+1; id++) {
    cout<<"======================="<<endl;
    //reading point cloud and convert to depth

    cout<<"Panorama "<<id<<endl;
    Panorama panorama;
    panorama.Init(file_io, id);

    PointCloud curpc;
    cout<<"reading point cloud..."<<endl;
    curpc.Init(file_io, id);
    curpc.ToGlobal(file_io, id);
    
    //Get depthmap
    cout<<"Processing depth map..."<<endl;
    DepthFilling depth;
    depth.Init(curpc, panorama);
    depth.SaveDepthmap("./depth.png");
    depth.fill_hole(panorama);
    depth.SaveDepthmap("./depth_denoise.png");
    
    Mat pan = panorama.GetRGBImage().clone();
    SLIC slic;
    int imgwidth = pan.cols;
    int imgheight = pan.rows;
    int numlabels(0);
    vector<int> labels(imgwidth*imgheight);
    vector<unsigned int>imagebuffer;
    MatToImagebuffer(pan, imagebuffer);

    cout<<"Performing SLICO Superpixel..."<<endl;
    slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[0],numlabels,FLAGS_label_num,0.0);
    slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[0],imgwidth,imgheight);

    cout<<"numlabels: "<<numlabels<<endl;

    //project point_cloud onto panorama
    PointCloud curob;
    string objectcloudpath = file_io.GetObjectPointClouds(id);
    curob.Init(objectcloudpath);
    curob.ToGlobal(file_io, id);
    
    Mat out;
    ImagebufferToMat(imagebuffer, imgwidth, imgheight, out);
    char buffer[100];
    sprintf(buffer,"%s/SLIC%03d.png",file_io.GetDataDirectory().c_str(),id);
    imwrite(buffer,out);
    waitKey(10);
  }

  return 0;
}
