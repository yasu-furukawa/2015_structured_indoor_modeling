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
DEFINE_int32(label_num,12000,"Number of superpixel");

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

  //read and group object point cloud
  vector <PointCloud> objectcloud;
  vector <vector <vector<int> > >objectgroup;
  cout <<"Reading object pointcloud!"<<endl;
  ReadObjectCloud(file_io, objectcloud, objectgroup);
    
  startid = 0;
  endid = 1;
  for (int id=startid; id<endid; id++) {
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

    sprintf(buffer,"superpixel/SLIC%03d.txt",id);
    ifstream labelin(buffer, ios::binary);
    if(!labelin.is_open()){
	cout<<"Performing SLICO Superpixel..."<<endl;
	Mat pan = panorama.GetRGBImage().clone();
	SLIC slic;
	vector<unsigned int>imagebuffer;
	MatToImagebuffer(pan, imagebuffer);
	slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[0],numlabels,FLAGS_label_num,0.0);
	slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[0],imgwidth,imgheight);

	sprintf(buffer,"superpixel/SLIC%03d.txt",id);
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


    ////////////////////////////////////////////////////
    Vector3d curpancenter = panorama.GetCenter();
    cout<<"panorama center:"<<curpancenter.transpose()<<endl;
    
    for(int roomid=0;roomid<objectcloud.size();roomid++){
      cout<<"room center:"<<objectcloud[roomid].GetCenter().transpose() <<endl;
      Vector3d center_dis = curpancenter - objectcloud[roomid].GetCenter();
      cout << "id:"<<id<<' '<<"roomid:"<<roomid<<' '<<"distance:"<<center_dis.norm()<<endl;
      
    }














    

    
//     //project point_cloud onto panorama
//     PointCloud curob;
//     cout<<"Reading object ply..."<<endl;
//     string objectcloudpath = file_io.GetObjectPointClouds(id);
//     cout << objectcloudpath<<endl;
//     curob.Init(objectcloudpath);
//     cout<<"curob.num_object: " << curob.GetNumObjects()<<endl;
    
//     vector <vector<int> >labelgroup;
//     vector <Vector3d> averageRGB;
//     labelTolabelgroup(labels, panorama, labelgroup, averageRGB, numlabels);
 
//     vector <vector <int> >objectgroup;
//     cout<<"Grouping objects..."<<endl;
//     int objectnum = groupObject(curob, objectgroup);
//     cout<<"Number of object:"<<objectnum<<endl;


//     //debug the projection
// #if 0
//     Vector3d temp(-3854.73,2324.54,-610.86);
//     Vector2d tempRGB = panorama.Project(temp);
//     int templabel = labels[(int)tempRGB[1] * imgwidth + (int)tempRGB[0]];
//     Mat tempMat(imgheight,imgwidth,CV_8UC3,Scalar(0,0,0));
//     for(int i=0;i<labelgroup[templabel].size();i++){
//       int tempx = labelgroup[templabel][i] % imgwidth;
//       int tempy = labelgroup[templabel][i] / imgwidth;
//       tempMat.at<Vec3b>(tempy,tempx) = Vec3b(255,255,255);
//     }
//     imwrite("ProjectionTest.png",tempMat);
//     waitKey(10);
// #endif
    

//     //get super pixel topology
//     cout<<"Getting pairwise structure..."<<endl;
//     map<pair<int,int>,int> pairmap;
//     pairSuperpixel(labels, imgwidth, imgheight, pairmap);
    
//     cout<<"Getting superpixel confidence.."<<endl;
//     vector<int>superpixelConfidence;
    
//     for(int groupid = 0;groupid <objectgroup.size(); ++groupid){
//       cout<<"For object: "<<groupid<<endl;
//       getSuperpixelLabel(curob, objectgroup[groupid], panorama, depth.GetDepthmap(), labels,labelgroup, superpixelConfidence, numlabels);
      
// #if 1
//       //save the mask
//       int minc = *min_element(superpixelConfidence.begin(), superpixelConfidence.end());
//       int maxc = *max_element(superpixelConfidence.begin(), superpixelConfidence.end());

//       Mat outmask = panorama.GetRGBImage().clone();
//       for(int i=0;i<imgwidth*imgheight;++i){
// 	int x = i % imgwidth;
// 	int y = i / imgwidth;
// 	int curconfidence =(int)((float) (superpixelConfidence[labels[i]] - minc) / (float)(maxc - minc) * 255.0);
// 	//if(superpixelConfidence[labels[i]] < 60)
// 	//outmask.at<Vec3b>(y,x) = Vec3b(0,0,0);
// 	Vec3b curpix((uchar)curconfidence,(uchar)curconfidence,(uchar)curconfidence);
// 	outmask.at<Vec3b>(y,x) = curpix;
//       }
//       sprintf(buffer,"object_project/objectmask%03d_object%03d.png",id, groupid);
//       imwrite(buffer, outmask);
//       waitKey(10);
// #endif


//       //MRF Optimize
//       vector <int> superpixelLabel(numlabels);
//       MRFOptimizeLabels(superpixelConfidence, pairmap, averageRGB,  0.1, superpixelLabel);

//       //save optimize result
//       Mat optimizeout = panorama.GetRGBImage().clone();
//       for(int y=0;y<imgheight;y++){
// 	for(int x=0;x<imgwidth;x++){
// 	  int curlabel = superpixelLabel[labels[y*imgwidth + x]];
// 	  Vec3b curpix = optimizeout.at<Vec3b>(y,x);
// 	  if(curlabel == 1){
// 	      curpix[0] = 255;
// 	  }
// 	  else
// 	      curpix[2] = 255;
// 	  optimizeout.at<Vec3b>(y,x) = curpix;
// 	}
//       }
//       sprintf(buffer,"object_project/optimize%03d_obj%03d.png",id,groupid);
//       imwrite(buffer,optimizeout);
//       waitKey(10);
//     }//for groupid
  }

  return 0;
}

