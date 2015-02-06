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
#include <algorithm>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

DEFINE_string(config_path,"lumber.configuration","Path to the configuration file");
DEFINE_int32(label_num,6000,"Number of superpixel");

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

    int imgwidth = panorama.Width();
    int imgheight = panorama.Height();
    int numlabels(0);
    vector<int> labels(imgwidth*imgheight);
    char buffer[100];

    sprintf(buffer,"SLIC%03d.txt",id);
    ifstream labelin(buffer, ios::binary);
    if(!labelin.is_open()){
	cout<<"Performing SLICO Superpixel..."<<endl;
	Mat pan = panorama.GetRGBImage().clone();
	SLIC slic;
	vector<unsigned int>imagebuffer;
	MatToImagebuffer(pan, imagebuffer);
	slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[0],numlabels,FLAGS_label_num,0.0);
	slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[0],imgwidth,imgheight);

	sprintf(buffer,"SLIC%03d.txt",id);
	slic.SaveSuperpixelLabels(&labels[0],imgwidth,imgheight,numlabels," ",string(buffer));
	cout<<"numlabels: "<<numlabels<<endl;
	Mat out;
	ImagebufferToMat(imagebuffer, imgwidth, imgheight, out);
	//sprintf(buffer,"%s/SLIC%03d.png",file_io.GetDataDirectory().c_str(),id);
	sprintf(buffer,"SLIC%03d.png",id); 
	imwrite(buffer,out);
	waitKey(10);
    }else{
	labelin.read((char*)&numlabels, sizeof(int));
	labels.resize(imgwidth * imgheight);
	for(int i=0;i<labels.size();i++){
	    labelin.read((char*)&labels[i], sizeof(int));
	}
	labelin.close();
    }
    
    //project point_cloud onto panorama
    PointCloud curob;
    cout<<"Reading object ply..."<<endl;
    string objectcloudpath = file_io.GetObjectPointClouds(id);
    cout << objectcloudpath<<endl;
    curob.Init(objectcloudpath);
    
    vector <vector<int> >labelgroup;
    labelTolabelgroup(labels, labelgroup, numlabels);
 
    vector <int> superpixelConfidence(numlabels);
    vector <vector <int> >objectgroup;
    cout<<"Grouping objects..."<<endl;
    int objectnum = groupObject(curob, objectgroup);
    cout<<"Number of object:"<<objectnum<<endl;
    
    cout<<"Getting superpixel confidence.."<<endl;
    getSuperpixelLabel(curob, panorama, depth.GetDepthmap(), labels,labelgroup, superpixelConfidence, numlabels);
#if 1
    //save the mask
    int minc = *min_element(superpixelConfidence.begin(), superpixelConfidence.end());
    int maxc = *max_element(superpixelConfidence.begin(), superpixelConfidence.end());

    Mat outmask = panorama.GetRGBImage().clone();
    for(int i=0;i<imgwidth*imgheight;++i){
	int x = i % imgwidth;
	int y = i / imgwidth;
	int curconfidence =(int)((float) (superpixelConfidence[labels[i]] - minc) / (float)(maxc - minc) * 255.0);
	Vec3b curpix((uchar)curconfidence,(uchar)curconfidence,(uchar)curconfidence);
	outmask.at<Vec3b>(y,x) = curpix;
    }
    sprintf(buffer,"objectmask%03d.png",id);
    imwrite(buffer, outmask);
    waitKey(10);
#endif
   
  }

  return 0;
}

