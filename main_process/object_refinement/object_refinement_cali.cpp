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

DEFINE_int32(label_num,10000,"Number of superpixel");
DEFINE_double(smoothness_weight,0.10,"Weight of smoothness term");
DEFINE_int32(start_id, 0,"Start id");
DEFINE_int32(end_id,0, "End id");

int main(int argc, char **argv){
#ifdef __APPLE__
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
    const int &endid = FLAGS_end_id;

    clock_t start,end;
    start = clock();
    //////////////////////////////////////
    //Init Panoramas
    //objectgroup: room->object->points
    //objectvolume: room->object volume
    vector <PointCloud> objectcloud;
    vector <PointCloud> backgroundCloud;
    vector <vector <vector<int> > >objectgroup;
    vector <vector <double> > objectvolume;
    vector <Panorama> panorama;
    vector <vector <int> >labels;
    vector <int> numlabels;
    vector <vector<int> >superpixelLabel(endid - startid + 1); //label of superpixel for each panorama
    vector <vector<vector<int> > >labelgroup(endid - startid + 1);
    vector < vector<list<PointCloud> > > objectlist; //room->object->object part
    vector <vector<list<PointCloud> > > input_objectlist;

    cout<<"Init..."<<endl;
    int imgheight, imgwidth;
    initPanorama(file_io, panorama, labels, FLAGS_label_num, numlabels, imgwidth, imgheight, startid, endid);
    ReadObjectCloud(file_io, objectcloud, objectgroup, objectvolume);

    objectlist.resize(objectcloud.size());
    input_objectlist.resize(objectcloud.size());

    
 
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
		getSuperpixelConfidence(objectcloud[roomid], objectgroup[roomid][groupid],panorama[curid], labels[curid], labelgroup[curid],pairmap, superpixelConfidence[groupid], numlabels[curid],5);
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
	    backProjectObject(panorama[curid], objectcloud[roomid], objectgroup[roomid], superpixelLabel[curid], labelgroup[curid], objectlist[roomid], panid, roomid);
	}
    }
    mergeObject(objectlist, objectcloud, resultCloud);
    /////////////////////////////    
    cout<<endl<<"All done! Saving result..."<<endl;

    for(int roomid=0; roomid<resultCloud.size(); roomid++){

	for(int i=0; i<resultCloud[roomid].GetNumObjects(); i++){
	    PointCloud curob;
	    vector<structured_indoor_modeling::Point>obpts;
	    resultCloud[roomid].GetObjectPoints(i, obpts);
	    curob.AddPoints(obpts);
	    vector<double>bbox = curob.GetBoundingbox();
	    double areaXY = (bbox[1] - bbox[0]) * (bbox[3] - bbox[2]);
	    double density = (double)curob.GetNumPoints() / areaXY;
	    sprintf(buffer,"temp/object_room%03d_object%03d.ply",roomid,i);
	    curob.Write(string(buffer));
	}
	cout<<"Cleaning room "<<roomid<<"..."<<endl;
	cleanObjects(resultCloud[roomid], 1e5);
	cout<<"Object num after cleaning: "<<resultCloud[roomid].GetNumObjects()<<endl;
	string savepath = file_io.GetRefinedObjectClouds(roomid);
	cout<<"Saving "<<savepath<<endl;
	resultCloud[roomid].Write(savepath);
    }
    end = clock();
    cout<<"Total time usage: "<<(end - start) / 1000000<<"s"<<endl;

    return 0;
}

