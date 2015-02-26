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
DEFINE_int32(label_num,20000,"Number of superpixel");
DEFINE_double(smoothness_weight,0.1,"Weight of smoothness term");

Vec3b colortable[] = {Vec3b(255,0,0), Vec3b(0,255,0), Vec3b(0,0,255), Vec3b(255,255,0), Vec3b(255,0,255), Vec3b(0,255,255), Vec3b(128,0,0), Vec3b(0,128,0), Vec3b(0,0,128), Vec3b(128,128,0), Vec3b(128,0,128), Vec3b(0,128,128), Vec3b(255,128,128),Vec3b(128,255,128),Vec3b(128,128,255)};


int main(int argc, char **argv){

  gflags::ParseCommandLineFlags(&argc,&argv,true);
  if(! (FLAGS_config_path.length() > 0)){
    cout<<"Usage: Object_hole_filling /path to your configuration file"<<endl;
  }
  //get path to data
  char pathtodata[100];
  char buffer[100];
  int startid, endid;
  ifstream confin(FLAGS_config_path.c_str());
  confin.getline(pathtodata,100);
  confin>>startid>>endid;
  confin.close();
  string pathtodata_s(pathtodata);
  FileIO file_io(pathtodata_s);

  //////////////////////////////////////
  //read and group object point cloud
  vector <PointCloud> objectcloud;
  vector <PointCloud> backgroundCloud;
  vector <vector <vector<int> > >objectgroup;
  vector <vector <double> > objectvolume;
  cout <<"Reading object pointcloud!"<<endl;
  ReadObjectCloud(file_io, objectcloud, objectgroup, objectvolume);

  //////////////////////////////////////
  PointCloud resultCloud;
  
  //////////////////////////////////////
  for (int id=startid; id<endid; id++) {
    cout<<"======================="<<endl;
    //reading point cloud and convert to depth

    cout<<"Panorama "<<id<<endl;
    Panorama panorama;
    panorama.Init(file_io, id);

    PointCloud curpc;
    cout<<"reading pnaorama  point cloud..."<<endl;
    curpc.Init(file_io, id);
    curpc.ToGlobal(file_io, id);
    
    //Get depthmap
    cout<<"Processing depth map..."<<endl;
    DepthFilling depth;
    sprintf(buffer,"depth/panorama%03d.depth",id);
    if(!depth.ReadDepthFromFile(string(buffer))){
	depth.Init(curpc, panorama);
	depth.fill_hole(panorama);
	depth.SaveDepthFile(string(buffer));
    }
    sprintf(buffer,"depth/panoramaDepth%03d.png",id);
    depth.SaveDepthmap(string(buffer));

    int imgwidth = panorama.Width();
    int imgheight = panorama.Height();
    int numlabels(0);
    vector<int> labels(imgwidth*imgheight);
    

    sprintf(buffer,"superpixel/SLIC%03d",id);
    ifstream labelin(buffer, ios::binary);
    if(!labelin.is_open()){
	cout<<"Performing SLICO Superpixel..."<<endl;
	Mat pan = panorama.GetRGBImage().clone();
	SLIC slic;
	vector<unsigned int>imagebuffer;
	MatToImagebuffer(pan, imagebuffer);
	slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[0],numlabels,FLAGS_label_num,0.0);
	slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[0],imgwidth,imgheight);

	sprintf(buffer,"superpixel/SLIC%03d",id);
	slic.SaveSuperpixelLabels(&labels[0],imgwidth,imgheight,numlabels," ",string(buffer));
	cout<<"numlabels: "<<numlabels<<endl;
	Mat out;
	ImagebufferToMat(imagebuffer, imgwidth, imgheight, out);
	//sprintf(buffer,"%s/SLIC%03d.png",file_io.GetDataDirectory().c_str(),id);
	sprintf(buffer,"SLIC%03d.png",id); 
	imwrite(buffer,out);
	waitKey(10);
    }else{
	cout <<"Reading superpixel from file"<<endl;
	labelin.read((char*)&numlabels, sizeof(int));
	labels.resize(imgwidth * imgheight);
	for(int i=0;i<labels.size();i++){
	    labelin.read((char*)&labels[i], sizeof(int));
	}
	labelin.close();
    }

    vector <vector<int> >labelgroup;
    vector <Vector3d> averageRGB;
    labelTolabelgroup(labels, panorama, labelgroup, averageRGB, numlabels);
    cout<<"Getting pairwise structure..."<<endl;
    map<pair<int,int>,int> pairmap;
    pairSuperpixel(labels, imgwidth, imgheight, pairmap);
    
    ////////////////////////////////////////////////////
    //get the superpixel confidence for each object and background
    
    for(int roomid = 0; roomid < objectcloud.size() ;roomid++){
      cout<<"--------------------------"<<endl;
      cout<<"room "<<roomid<<endl;
      vector <vector<double> >superpixelConfidence(objectgroup[roomid].size());
      cout<<"Get superpixel confidence"<<endl;
      for(int groupid = 0;groupid<objectgroup[roomid].size();groupid++){
	getSuperpixelConfidence(objectcloud[roomid], objectgroup[roomid][groupid],panorama, depth.GetDepthmap(), labels, labelgroup, superpixelConfidence[groupid], numlabels);
      }
#if 0
      cout<<"saving mask..."<<endl;
      //save the mask
      double minc = 1e100;
      double maxc = -1e100;

      for(int i=0;i<superpixelConfidence.size();i++){
	  for(int j=0;j<superpixelConfidence[i].size();j++){
	      minc = min(superpixelConfidence[i][j], minc);
	      maxc = max(superpixelConfidence[i][j], maxc);
	  }
      }
      Mat outmask(imgheight,imgwidth,CV_8UC3, Scalar(0,0,0));
      for(int groupid = 0;groupid<objectgroup[roomid].size();groupid++){
	  int colorid = groupid % 15;
	  for(int i=0;i<imgwidth*imgheight;++i){
	      int x = i % imgwidth;
	      int y = i / imgwidth;
	      double curconfidence =(double) (superpixelConfidence[groupid][labels[i]] - minc) / (double)(maxc - minc);
	      double r = (double)colortable[colorid][0] * curconfidence * 4;
	      double g = (double)colortable[colorid][1] * curconfidence * 4;
	      double b = (double)colortable[colorid][2] * curconfidence * 4;
	      Vec3b curpix((uchar)r, (uchar)g, (uchar)b);
	      outmask.at<Vec3b>(y,x) += curpix;
	  }
      }
      sprintf(buffer,"object_project/objectmask_panorama%03d_room%03d.jpg",id, roomid);
      imwrite(buffer, outmask);
      waitKey(10);
#endif


      vector <int> superpixelLabel;
      cout<<"Optimizing..."<<endl;
      MRFOptimizeLabels_multiLayer(superpixelConfidence, pairmap, averageRGB, FLAGS_smoothness_weight, objectgroup[roomid].size(),superpixelLabel);

#if 1
      //save optimize result
      Mat optimizeout = panorama.GetRGBImage().clone();
      for(int y=0;y<imgheight;y++){
	  for(int x=0;x<imgwidth;x++){
	      int curlabel = superpixelLabel[labels[y*imgwidth + x]];
	      int colorid = curlabel;
	      Vec3b curpix;
	      if(colorid <= 15)
		  curpix = colortable[colorid] * 0.8 + optimizeout.at<Vec3b>(y,x)*0.2;
	      else
		  curpix = Vec3b((uchar)rand()%255,(uchar)rand()%255,(uchar)rand()%255);

	      optimizeout.at<Vec3b>(y,x) = curpix;
	  }
      }
      sprintf(buffer,"object_project/optimize_pan%03d_room%03d.png",id,roomid);
      imwrite(buffer,optimizeout);
      waitKey(10);
#endif

      //back project
      cout<<"Back projecting..."<<endl;
      BackProjectObject(panorama,depth.GetDepthmap(), superpixelLabel, labelgroup, resultCloud);
    }
  }

  /////////////////////////////
  cout<<endl<<"All done! Saving result..."<<endl;
  resultCloud.Write("result.ply");
  return 0;
}

