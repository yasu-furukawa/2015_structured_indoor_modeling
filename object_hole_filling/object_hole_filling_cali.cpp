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
#include "time.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

DEFINE_string(config_path,"lumber.configuration","Path to the configuration file");
DEFINE_int32(label_num,20000,"Number of superpixel");
DEFINE_double(smoothness_weight,0.10,"Weight of smoothness term");

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
    //Init Panoramas
    //objectgroup: room->object->points
    //objectvolume: room->object volume
    vector <PointCloud> objectcloud;
    vector <PointCloud> backgroundCloud;
    vector <vector <vector<int> > >objectgroup;
    vector <vector <double> > objectvolume;
    vector <Panorama> panorama;
    vector <DepthFilling> depth;
    vector <vector <int> >labels;
    vector <int> numlabels;
    vector <vector<int> >superpixelLabel(endid - startid + 1); //label of superpixel for each panorama
    vector <vector<vector<int> > >labelgroup(endid - startid + 1);

    cout<<"Init..."<<endl;
    int imgheight, imgwidth;
    initPanorama(file_io, panorama, labels, FLAGS_label_num, numlabels, depth, imgwidth, imgheight, startid, endid);
    ReadObjectCloud(file_io, objectcloud, objectgroup, objectvolume);
	 
    //////////////////////////////////////
    vector <PointCloud> resultCloud(objectcloud.size()); //object cloud per-room

    cout<<endl<<endl;
    //////////////////////////////////////
    for (int panid=startid; panid<=endid; panid++) {
	cout<<"==========================="<<endl<<"Panorama "<<panid<<endl;
	int curid = panid - startid;

	vector <Vector3d> averageRGB;
	labelTolabelgroup(labels[curid], panorama[curid], labelgroup[curid], averageRGB, numlabels[curid]);
	cout<<"Getting pairwise structure..."<<endl;
	map<pair<int,int>,int> pairmap;
	pairSuperpixel(labels[curid], imgwidth, imgheight, pairmap);
    
	////////////////////////////////////////////////////
	//get the superpixel confidence for each object and background
    	for(int roomid = 0; roomid < objectcloud.size() ;roomid++){
	    cout<<"--------------------------"<<endl;
	    cout<<"room "<<roomid<<endl;
	    cout<<"Get superpixel confidence"<<endl;

	    vector< vector<double> >superpixelConfidence(objectgroup[roomid].size());  //object->superpixel
	    for(int groupid = 0;groupid<objectgroup[roomid].size();groupid++){
		 getSuperpixelConfidence(objectcloud[roomid], objectgroup[roomid][groupid],panorama[curid], depth[curid], labels[curid], labelgroup[curid],pairmap, superpixelConfidence[groupid], numlabels[curid],5);
	    }
#if 0
	    saveConfidence(superpixelConfidence, labels[curid], imgwidth, imgheight, panid, roomid);
#endif
	    DepthFilling objectDepth;
	    cout<<"Optimizing..."<<endl;
	    cout<<"numlabel:"<<objectgroup[roomid].size()<<endl;
	    MRFOptimizeLabels_multiLayer(superpixelConfidence, pairmap, averageRGB, FLAGS_smoothness_weight, objectgroup[roomid].size(),superpixelLabel[curid]);
#if 1
	    saveOptimizeResult(panorama[curid], superpixelLabel[curid], labels[curid], panid,roomid);
#endif
	    BackProjectObject(panorama[curid], depth[curid], objectcloud[roomid], objectgroup[roomid], superpixelLabel[curid], labelgroup[curid], resultCloud[roomid], roomid);
	}
    }

    /////////////////////////////    
    cout<<endl<<"All done! Saving result..."<<endl;

    for(int roomid=0; roomid<resultCloud.size(); roomid++){
    	 // cout<<"Merging close points..."<<endl;
    	 // cout<<"Before merging: "<<resultCloud[roomid].GetNumPoints()<<endl;
    	 // clock_t start,end;
    	 // start = clock();
    	 // mergeVertices(resultCloud[roomid], 1024);
    	 // end = clock();
    	 // cout<<"done. Time: "<<end - start<<endl;
    	 // cout<<"After merging: "<<resultCloud[roomid].GetNumPoints()<<endl;
    	 string savepath = file_io.GetRefinedObjectClouds(roomid);
    	 cout<<"Saving "<<savepath<<endl;
    	 resultCloud[roomid].Write(savepath);
    }

    return 0;
}

