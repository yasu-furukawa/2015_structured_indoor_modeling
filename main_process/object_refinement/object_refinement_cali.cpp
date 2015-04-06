#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <string>
#include <gflags/gflags.h>
#include "../../base/file_io.h"
#include "../../base/point_cloud.h"
#include "../../base/panorama.h"
#include <vector>
#include <typeinfo>
#include "object_refinement.h"
#include "depth_filling.h"
#include <algorithm>
#include "time.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;


DEFINE_int32(label_num,12000,"Number of superpixel");
DEFINE_double(smoothness_weight,0.15,"Weight of smoothness term");
DEFINE_int32(start_id, 0,"Start id");
DEFINE_int32(end_id,-1, "End id");
DEFINE_int32(nsmooth, 3, "Iterations of smoothing");
DEFINE_bool(recompute, false, "Recompute superpixel");

bool compare_by_z(const structured_indoor_modeling::Point &pt1, const structured_indoor_modeling::Point &pt2){
     return pt1.position[2] < pt2.position[2];
}

int main(int argc, char **argv){
#if 0
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

    if (argc < 2) {
	cerr << "Usage: " << argv[0] << " data_directory" << endl;
	return 1;
    }

    //get path to data

    char buffer[1024];
    FileIO file_io(argv[1]);

    const int &startid = FLAGS_start_id;
    const int &endid = GetNumPanoramas(file_io) - 1;

    clock_t start,end;
    start = clock();
    //////////////////////////////////////
    //Init Panoramas
    //objectgroup: room->object->points
    //objectvolume: room->object volume
    vector <PointCloud> objectcloud;
    vector <PointCloud> backgroundCloud;
    vector <vector <vector<int> > >objectgroup;
    vector <Panorama> panorama;
    vector <vector <int> >labels;
    vector <int> numlabels;
    vector <vector<int> >superpixelLabel(endid - startid + 1); //label of superpixel for each panorama
    vector <vector<vector<int> > >labelgroup(endid - startid + 1);
    vector < vector<list<PointCloud> > > objectlist; //room->object->object part
    vector<DepthFilling> depth(endid - startid + 1);
    vector< vector< vector <int> > > object_panorama;  // room->object->list_of_panoramaid

    cout<<"Init..."<<endl;
    int imgheight, imgwidth;
    initPanorama(file_io, panorama, labels, FLAGS_label_num, numlabels,depth, imgwidth, imgheight, startid, endid, FLAGS_recompute);
    ReadObjectCloud(file_io, objectcloud, objectgroup);
    object_panorama.resize(objectcloud.size());

    for(int roomid=0; roomid<objectcloud.size(); roomid++){
    	 // for(int objid=0; objid<objectgroup[roomid].size(); objid++){
    	 //      sprintf(buffer,"temp/object_room%03d_obj%03d.ply",roomid,objid);
    	 //      objectcloud[roomid].WriteObject(string(buffer), objid);
    	 // }
    	cout<<"---------------------"<<endl;
    	cout<<"Room "<<roomid<<endl;
    	getObjectColor(objectcloud[roomid], panorama, objectgroup[roomid] ,object_panorama[roomid],roomid);
	cleanObjects(objectcloud[roomid], objectgroup[roomid]);

	//sort point according to z
	for(int objid=0; objid<objectgroup[roomid].size(); objid++){
	     vector<structured_indoor_modeling::Point>sort_array;
	     for(const auto&ptid :objectgroup[roomid][objid])
		  sort_array.push_back(objectcloud[roomid].GetPoint(ptid));
	     sort(sort_array.begin(), sort_array.end(), compare_by_z);
	     for(int i=0; i<objectgroup[roomid][objid].size(); ++i)
		  objectcloud[roomid].GetPoint(objectgroup[roomid][objid][i]) = sort_array[i];
	}

	//Smoothing
	cout<<"Smoothing object... "<<flush;
	const int kNumNeighbors = 8;
	vector<vector<int> >neighbors;
	cout<<"Set neighbors...";
	SetNeighbors(objectcloud[roomid].GetPointData(), kNumNeighbors, &neighbors);
	cout<<"done!"<<endl;
	cout<<"Smoothing..."<<flush;
	for(int t=0;t<FLAGS_nsmooth;t++)
	     SmoothObjects(neighbors, &objectcloud[roomid].GetPointData());
	cout<<"done!"<<endl;
	cout<<"Saving "<<file_io.GetRefinedObjectClouds(roomid)<<endl;
	
    	objectcloud[roomid].Write(file_io.GetRefinedObjectClouds(roomid));
    }

    

    objectlist.resize(objectcloud.size());
    
 
//     //////////////////////////////////////
//     vector <PointCloud> resultCloud(objectcloud.size()); //object cloud per-room

//     cout<<endl<<endl;


//     //////////////////////////////////////
//     for (int panid=startid; panid<=endid; panid++) {

// 	cout<<"==========================="<<endl<<"Panorama "<<panid<<endl;
// 	int curid = panid - startid;

// 	vector <Vector3d> averageRGB;
// 	labelTolabelgroup(labels[curid], panorama[curid], labelgroup[curid], averageRGB, numlabels[curid]);
// 	cout<<"Getting pairwise structure..."<<endl;
// 	map<pair<int,int>,int> pairmap;
// 	pairSuperpixel(labels[curid], imgwidth, imgheight, pairmap);
    
// 	////////////////////////////////////////////////////
// 	//get the superpixel confidence for each object and background
//     	for(int roomid = 0; roomid < objectcloud.size() ;roomid++){
// 	    cout<<"--------------------------"<<endl;
// 	    cout<<"room "<<roomid<<endl;
// 	    cout<<"Get superpixel confidence"<<endl;

// 	    vector< vector<double> >superpixelConfidence(objectgroup[roomid].size());  //object->superpixel
// 	    for(int groupid = 0;groupid<objectgroup[roomid].size();groupid++){
// 		 getSuperpixelConfidence(objectcloud[roomid], objectgroup[roomid][groupid],panorama[curid], labels[curid], labelgroup[curid],pairmap,depth[curid], superpixelConfidence[groupid], numlabels[curid],1);
// 	    }
// #if 1
// 	    saveConfidence(superpixelConfidence, labels[curid], imgwidth, imgheight, panid, roomid);
// #endif
// 	    DepthFilling objectDepth;
// 	    cout<<"Optimizing..."<<endl;
// 	    cout<<"numlabel:"<<objectgroup[roomid].size()<<endl;
// 	    MRFOptimizeLabels_multiLayer(superpixelConfidence, pairmap, averageRGB, FLAGS_smoothness_weight, objectgroup[roomid].size(),superpixelLabel[curid]);
// #if 1
// 	    saveOptimizeResult(panorama[curid], superpixelLabel[curid], labels[curid], panid,roomid);
//#endif
//	    backProjectObject(panorama[curid], objectcloud[roomid], objectgroup[roomid], superpixelLabel[curid], labelgroup[curid], objectlist[roomid], panid, roomid);
//	}
//    }
//      mergeObject(objectlist, objectcloud, resultCloud);
//     // /////////////////////////////    
//      cout<<endl<<"All done! Saving result..."<<endl;

//     for(int roomid=0; roomid<resultCloud.size(); roomid++){

//     	for(int i=0; i<resultCloud[roomid].GetNumObjects(); i++){
//     	    PointCloud curob;
//     	    vector<structured_indoor_modeling::Point>obpts;
//     	    resultCloud[roomid].GetObjectPoints(i, obpts);
//     	    curob.AddPoints(obpts);
//     	    vector<double>bbox = curob.GetBoundingbox();
//     	    double areaXY = (bbox[1] - bbox[0]) * (bbox[3] - bbox[2]);
//     	    double density = (double)curob.GetNumPoints() / areaXY;
//     	    sprintf(buffer,"temp/object_room%03d_object%03d.ply",roomid,i);
//     	    curob.Write(string(buffer));
//     	}
//     	cout<<"Cleaning room "<<roomid<<"..."<<endl;
//     	cleanObjects(resultCloud[roomid], 1e5);
//     	cout<<"Object num after cleaning: "<<resultCloud[roomid].GetNumObjects()<<endl;
//     	string savepath = file_io.GetRefinedObjectClouds(roomid);
//     	cout<<"Saving "<<savepath<<endl;
//     	resultCloud[roomid].Write(savepath);
//     }
    end = clock();
    cout<<"Total time usage: "<<(end - start) / 1000000<<"s"<<endl;

    return 0;
}

