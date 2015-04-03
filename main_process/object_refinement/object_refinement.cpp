#include "object_refinement.h"
#include "SLIC/SLIC.h"
#include <numeric>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

#define PI 3.1415927
#define COLORTABLE_LENGTH 7
#define ERODE_THRES 1
//Vec3b colortable[] = {Vec3b(255,0,0), Vec3b(0,255,0), Vec3b(0,0,255), Vec3b(255,255,0), Vec3b(255,0,255), Vec3b(0,255,255), Vec3b(128,0,0), Vec3b(0,128,0), Vec3b(0,0,128), Vec3b(128,128,0), Vec3b(128,0,128), Vec3b(0,128,128), Vec3b(255,128,128),Vec3b(128,255,128),Vec3b(128,128,255),Vec3b(128,128,128)};

Vec3b colortable[] = {Vec3b(255,0,0), Vec3b(0,255,0), Vec3b(0,0,255), Vec3b(255,255,0), Vec3b(255,0,255), Vec3b(0,255,255),Vec3b(255,255,255)};

void initPanorama(const FileIO &file_io, vector<Panorama>&panorama, vector< vector<int> >&labels, const int expected_num, vector<int>&numlabels, vector<DepthFilling>&depth, int &imgwidth, int &imgheight, const int startid, const int endid, const bool recompute){
    cout<<"Init panorama..."<<endl;

    char buffer[100];
    
    panorama.resize(endid - startid + 1);
    labels.resize(endid - startid + 1);
    numlabels.resize(endid - startid + 1);
    
    for(int id=startid; id<=endid; id++){
	int curid = id - startid;
	cout<<"Panorama "<<id<<endl;
	panorama[curid].Init(file_io, id);
	panorama[curid].MakeOnlyBackgroundBlack();
           	
	imgwidth = panorama[curid].Width();
	imgheight = panorama[curid].Height();

    
	// //Get depthmap
	// cout<<"Processing depth map..."<<endl;
	// sprintf(buffer,"depth/panorama%03d.depth",id);
	// if(!depth[curid].ReadDepthFromFile(string(buffer))){
	//      PointCloud curpc;
	//      cout<<"reading pnaorama  point cloud..."<<endl;
	//      curpc.Init(file_io, id);
	//      curpc.ToGlobal(file_io, id);

	//      depth[curid].Init(curpc, panorama[curid]);
	//      depth[curid].fill_hole(panorama[curid]);
	//      depth[curid].SaveDepthFile(string(buffer));
	// }
	// sprintf(buffer,"depth/panoramaDepth%03d.png",id);
	// depth[curid].SaveDepthmap(string(buffer));

	
    // 	labels[curid].resize(imgwidth*imgheight);

    // 	ifstream labelin(file_io.GetSuperPixelFile(id).c_str(), ios::binary);
    // 	if(!labelin.is_open() || recompute){
    // 	    cout<<"Performing SLICO Superpixel..."<<endl;
    // 	    Mat pan = panorama[curid].GetRGBImage().clone();
    // 	    SLIC slic;
    // 	    vector<unsigned int>imagebuffer;
    // 	    MatToImagebuffer(pan, imagebuffer);
    // 	    slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[curid][0],numlabels[curid],expected_num,0.0);
    // 	    slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[curid][0],imgwidth,imgheight);
    // 	    sprintf(buffer,"superpixel/SLIC%03d",id);
    // 	    slic.SaveSuperpixelLabels(&labels[curid][0],imgwidth,imgheight,numlabels[curid]," ",file_io.GetSuperPixelFile(id));
    // 	    cout<<"numlabels: "<<numlabels[curid]<<endl;
    // 	    Mat out;
    // 	    ImagebufferToMat(imagebuffer, imgwidth, imgheight, out);
    // 	    sprintf(buffer,"superpixel/SLIC%03d.png",id); 
    // 	    imwrite(buffer,out);
    // 	    waitKey(10);
    // 	}else{
    // 	    cout <<"Reading superpixel from file"<<endl;
    // 	    labelin.read((char*)&numlabels[curid], sizeof(int));
    // 	    for(int i=0;i<labels[curid].size();i++){
    // 		labelin.read((char*)&labels[curid][i], sizeof(int));
    // 	    }
    // 	    labelin.close();
    // 	}
      cout<<endl;
    }
}

void AllRange(vector<int>&array, vector<vector<int> >&result, int k, int m){
    if(k==m){
	result.push_back(array);
    }
    else{
	for(int i=k; i<=m; i++){
	    swap(array[k],array[i]);
	    AllRange(array, result, k+1, m);
	    swap(array[k], array[i]);
	}
    }
}

void getObjectColor(PointCloud &objectcloud,const vector<Panorama>&panorama,const vector<vector<int> >&objectgroup, vector< vector<int> >&object_panorama, const int roomid){
    const double depth_margin = 50.0;
    const int min_overlap_points = 10;
    const int pansize = panorama.size();
    const double min_assigned_ratio = 0.98;
    const double max_averagedis = 5000.0;
    char buffer[100];

    vector<bool>assigned(objectcloud.GetNumPoints());
    vector<bool>is_used(pansize);
    vector<double>averagedis(pansize);

    vector<int>point_to_remove;

    for(int objid=0; objid<objectgroup.size(); objid++){
	for(const auto&v: objectgroup[objid])
	    assigned[v] = false;
	for(auto &v: averagedis)
	    v = 0.0;
	vector<vector<int> >point_list(pansize);
	//Get list of visible points of each panorama
	for(int panid=0; panid<pansize; panid++){
	    for(const auto& ptid: objectgroup[objid]){
		Vector3d curpt = objectcloud.GetPoint(ptid).position;
		double ptdepth = (curpt - panorama[panid].GetCenter()).norm();
		Vector2d RGB_pix = panorama[panid].Project(curpt);
		if(!panorama[panid].IsInsideRGB(RGB_pix))
		    continue;
		if(panorama[panid].GetRGB(RGB_pix).norm() == 0)
		    continue;
		Vector2d depth_pix = panorama[panid].RGBToDepth(RGB_pix);
		if(ptdepth < panorama[panid].GetDepth(depth_pix) + depth_margin){
		    point_list[panid].push_back(ptid);
		    averagedis[panid] += ptdepth;
		}
	    }
	    if(point_list[panid].size() != 0)
		averagedis[panid] /= (double)point_list[panid].size();
	    else
		averagedis[panid] = -1;
	}
	//Geeadily search for smallest set of panorama
	vector<int>pan_selected;
	for(int i=0; i<pansize; i++)
	    is_used[i] = false;
	while(true){
	    double totalcoverage = 0;
	    for(const auto&v: objectgroup[objid]){
		if(assigned[v])
		    totalcoverage += 1.0;
	    }
	    if(totalcoverage >= min_assigned_ratio * (double)objectgroup[objid].size())
		break;
	       
	    double max_score = 0;
	    int max_panid = -1;
	    for(int panid=0; panid<pansize; panid++){
		if(is_used[panid] || (averagedis[panid] == 0))
		    continue;
		double curcoveragegain = 0;
		for(const auto &ptid: point_list[panid]){
		    if(!assigned[ptid])
			curcoveragegain += 1.0;
		}
		if(curcoveragegain > max_score && averagedis[panid] < max_averagedis){
		    max_score = curcoveragegain;
		    max_panid = panid;
		}
	    }
	    if(max_panid == -1)
		break;
	    for(const auto&v: point_list[max_panid])
		assigned[v] = true;
	    is_used[max_panid] = true;
	    pan_selected.push_back(max_panid);
	}//while
#if 1
	cout<<"object "<<objid<<",used panorama: ";
	for(const auto&v: pan_selected)
	    cout<<v<<' ';
	cout<<endl;
#endif

	object_panorama.push_back(pan_selected);
	//assign color
	for(int ptid=0; ptid<objectcloud.GetNumPoints(); ptid++)
	    assigned[ptid] = false;
	for(const auto& panid: pan_selected){
	    
	    Mat panout = panorama[panid].GetRGBImage().clone();
	    
	    vector<Vector3f>color_src;
	    vector<Vector3f>color_tgt;
	    for(const auto& ptid: point_list[panid]){
		Vector3d curpt = objectcloud.GetPoint(ptid).position;
		Vector2d RGB_pix = panorama[panid].Project(curpt);
		
		panout.at<Vec3b>((int)RGB_pix[1], (int)RGB_pix[0])[0] = 255;
		panout.at<Vec3b>((int)RGB_pix[1], (int)RGB_pix[0])[1] /= 2;
		panout.at<Vec3b>((int)RGB_pix[1], (int)RGB_pix[0])[2] /= 2;
		
		Vector3f curColor = panorama[panid].GetRGB(RGB_pix);
		swap(curColor[0],curColor[2]);
		if(assigned[ptid]){
		    color_src.push_back(curColor);
		    color_tgt.push_back(objectcloud.GetPoint(ptid).color);
		}
	    }
	    
	    sprintf(buffer,"temp/room%03d_obj%03d_pan%03d.png", roomid, objid, panid);
	    imwrite(string(buffer), panout);
	    waitKey(10);

	    
	    Matrix3f colorTransform = Matrix3f::Identity();
	    if(color_src.size() > min_overlap_points)
		computeColorTransform(color_src, color_tgt, colorTransform);
	    for(const auto& ptid: point_list[panid]){
		if(assigned[ptid])
		    continue;
		Vector3d curpt = objectcloud.GetPoint(ptid).position;
		Vector2d RGB_pix = panorama[panid].Project(curpt);
		Vector3f curColor = panorama[panid].GetRGB(RGB_pix);
		swap(curColor[0], curColor[2]);
		Vector3f color_to_assigned =  colorTransform*curColor;
		
		if(color_to_assigned[0]<0||color_to_assigned[0]>255||
		   color_to_assigned[1]<0||color_to_assigned[1]>255||
		   color_to_assigned[2]<0||color_to_assigned[2]>255)
		    color_to_assigned = curColor;
		  
		objectcloud.SetColor(ptid, color_to_assigned);
		assigned[ptid] = true;
	    }
	    PointCloud curout;
	    vector<structured_indoor_modeling::Point>point_to_add;
	    for(const auto& ptid: point_list[panid])
	    	point_to_add.push_back(objectcloud.GetPoint(ptid));
	    curout.AddPoints(point_to_add);

	    sprintf(buffer,"panoramacloud/object_room%03d_obj%03d_pan%03d_ori.ply",roomid, objid, panid);
	    curout.Write(string(buffer));
	}

	//remove unassigned points
	for(const auto& ptid: objectgroup[objid]){
	    if(assigned[ptid] == false)
		point_to_remove.push_back(ptid);
	}
    }//for objid
     objectcloud.RemovePoints(point_to_remove);
}

Eigen::Vector3d Intersect(const structured_indoor_modeling::Point& lhs, const structured_indoor_modeling::Point& rhs) {
     // Ray is point = lhs.position + lhs.normal * d.
     // rhs.normal * (point - rhs.position)

     const Vector3d diff = lhs.position - rhs.position;
     const Vector3d diff_along_rhs_normal = rhs.normal.dot(diff) * rhs.normal;
     const Vector3d final_diff = lhs.normal.dot(diff_along_rhs_normal) * lhs.normal;
     return lhs.position - final_diff;
}

void MatToImagebuffer(const Mat image, vector<unsigned int>&imagebuffer){
    if(!image.data){
	cout << "invlid image"<<endl;
	exit(-1);
    }
    const int imgheight = image.rows;
    const int imgwidth = image.cols;
    imagebuffer.clear();
    imagebuffer.resize(imgheight * imgwidth);
    for(int y=0;y<imgheight;y++){
	for(int x=0;x<imgwidth;x++){
	    Vec3b curpix = image.at<Vec3b>(y,x);
	    int ind = y*imgwidth + x;
	    imagebuffer[ind] = (unsigned int)255*256*256*256 + (unsigned int)curpix[0]*256*256 + (unsigned int)curpix[1]*256 + (unsigned int)curpix[2];
	}
    }
}


void ImagebufferToMat(const vector <unsigned int>&imagebuffer,const int imgwidth,const int imgheight,  Mat& image){
    if(imagebuffer.size() != imgwidth * imgheight){
	cout << "Sizes don't agree!"<<endl;
	exit(-1);
    }
    image.release();
    image = Mat(imgheight,imgwidth,CV_8UC3);
    for(int y=0;y<imgheight;y++){
	for(int x=0;x<imgwidth;x++){
	    Vec3b curpix;
	    curpix[0] = imagebuffer[y*imgwidth+x] >> 16 & 0xff;
	    curpix[1] = imagebuffer[y*imgwidth+x] >> 8 & 0xff;
	    curpix[2] = imagebuffer[y*imgwidth+x] & 0xff;
	    image.at<Vec3b>(y,x) = curpix;
	}
    }
}

void saveConfidence(const vector< vector<double> >&superpixelConfidence, const vector<int>&labels, const int imgwidth, const int imgheight, const int id, const int roomid){
    cout<<"saving mask..."<<endl;
    //save the mask
    double minc = 1e100;
    double maxc = -1e100;
    char buffer[100];

    for(int i=0;i<superpixelConfidence.size();i++){
	for(int j=0;j<superpixelConfidence[i].size();j++){
	    minc = min(superpixelConfidence[i][j], minc);
	    maxc = max(superpixelConfidence[i][j], maxc);
	}
    }
    Mat outmask(imgheight,imgwidth,CV_8UC3, Scalar(0,0,0));
    for(int groupid = 0;groupid<superpixelConfidence.size();groupid++){
	int colorid = groupid % COLORTABLE_LENGTH;
	for(int i=0;i<imgwidth*imgheight;++i){
	    int x = i % imgwidth;
	    int y = i / imgwidth;
//	    double curconfidence =(double) (superpixelConfidence[groupid][labels[i]] - minc) / (double)(maxc - minc);
	    double curconfidence = (double)(superpixelConfidence[groupid][labels[i]]);
	    double r = (double)colortable[colorid][0] * curconfidence / 2;
	    double g = (double)colortable[colorid][1] * curconfidence / 2;
	    double b = (double)colortable[colorid][2] * curconfidence / 2;
	    Vec3b curpix((uchar)r, (uchar)g, (uchar)b);
	    outmask.at<Vec3b>(y,x) += curpix;
	}
    }
    sprintf(buffer,"object_project/objectmask_panorama%03d_room%03d.jpg",id, roomid);
    imwrite(buffer, outmask);
    waitKey(10);
}

void saveOptimizeResult(const Panorama &panorama, const vector<int>&superpixelLabel, const vector <int> &labels, const int id, const int roomid){
    //save optimize result
    const int imgwidth = panorama.Width();
    const int imgheight = panorama.Height();
    char buffer[100];
    
    Mat optimizeout = panorama.GetRGBImage().clone();
    for(int y=0;y<imgheight;y++){
	for(int x=0;x<imgwidth;x++){
	    int curlabel = superpixelLabel[labels[y*imgwidth + x]];
	    int colorid = curlabel % COLORTABLE_LENGTH;
	    Vec3b curpix;
	    curpix = colortable[colorid] * 0.8 + optimizeout.at<Vec3b>(y,x)*0.2;
	    optimizeout.at<Vec3b>(y,x) = curpix;
	}
    }
    sprintf(buffer,"object_project/optimize_room%03d_pan%03d.png",roomid,id);
    imwrite(buffer,optimizeout);
    waitKey(10);
}

void RGB2HSV(double r,double g,double b, double &h, double &s, double &v){
    double min = r, max = r;
    min = std::min(g,min); min = std::min(b,min);
    max = std::max(g,max); max = std::max(b,max);

    v = max;
    int delta = max-min;
    if(max !=0)
	s = delta / (float)max;
    else{
	s = 0;
	h = -1;
	return;
    }
    if(r == max)
	h = (g-b) / delta;
    else if(g == max)
	h = 2+(b-r)/delta;
    else
	h = 4+(r-g)/delta;
    h *= 60;
    if(h<0)
	h += 360;
}

void HSV2RGB(double h, double s, double v, double &r, double &g, double &b){
    int i;
    double f,p,q,t;
    if(s==0){
	r = v;
	g = v;
	b = v;
	return;
    }
    h /= 60;
    i = floor(h);
    f = h - i;
    p = v*(1-s);
    q = v*(1-s*f);
    t = v*(1-s*(1-f));

    switch(i){
    case 0:
	r = v; g = t; b = p;
	break;
    case 1:
	r = q; g = v; b = p;
	break;
    case 2:
	r = p; g = v; b = t;
	break;
    case 3:
	r = p; g = q; b = v;
	break;
    case 4:
	r = t; g = p; b = v;
	break;
    default:
	r = v; g = p; b = q;
	break;
    }
}

void labelTolabelgroup(const vector<int>& labels, const Panorama &panorama, vector< vector<int> >&labelgroup, vector< Vector3d >& averageRGB, int numgroup){
    int width = panorama.Width();
    int height = panorama.Height();
    labelgroup.resize(numgroup);
    averageRGB.resize(numgroup);
    for(auto &rgb: averageRGB)
	rgb.resize(3);
    for(int i=0;i<labels.size();i++){
	labelgroup[labels[i]].push_back(i);
    }
    for(int i=0;i<labelgroup.size();i++){
	averageRGB[i][0] = 0; averageRGB[i][1] = 0; averageRGB[i][2] = 0;
	for(int j=0;j<labelgroup[i].size();j++){
	    int curpix = labelgroup[i][j];
	    Vector3f curcolor = panorama.GetRGB(Vector2d((double)(curpix % width),  (double)(curpix / width)));
	    averageRGB[i][0] += curcolor[0];
	    averageRGB[i][1] += curcolor[1];
	    averageRGB[i][2] += curcolor[2];
	}
	averageRGB[i] = averageRGB[i] / (double)labelgroup[i].size();
    }
}

bool visibilityTest(const structured_indoor_modeling::Point &pt, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, int depthwidth){
    Vector3d curpt = pt.position;
    Vector3d localpt = panorama.GlobalToLocal(curpt);
    Vector2d pixel = panorama.Project(curpt);
    Vector2d depth_pixel = panorama.RGBToDepth(pixel);
    double curdepth = std::sqrt(localpt[0]*localpt[0] + localpt[1]*localpt[1] + localpt[2]*localpt[2]);
    if(curdepth > depthmap[(int)depth_pixel[1] * depthwidth + (int)depth_pixel[0]] + 0.1)
	return false;
    return true;
}


int groupObject(const PointCloud &point_cloud, vector <vector<int> >&objectgroup){
    objectgroup.resize(point_cloud.GetNumObjects());
    for(int i=0;i<point_cloud.GetNumPoints();++i){
	structured_indoor_modeling::Point curpt = point_cloud.GetPoint(i);
	int curid = curpt.object_id;
	objectgroup[curid].push_back(i);
    }
    return (int)objectgroup.size();
}



void getSuperpixelConfidence(const PointCloud &point_cloud,const vector<int> &objectgroup,  const Panorama &panorama, const vector<int> &superpixel,const vector< vector<int> >&labelgroup, const map<pair<int,int>,int>&pairmap, const DepthFilling& depthmap,  vector <double> &superpixelConfidence, const int superpixelnum, const int erodeiter){
    if(superpixelConfidence.size() > 0)
	superpixelConfidence.clear();
    superpixelConfidence.resize(superpixelnum);
    
    for(int i=0;i<superpixelConfidence.size();++i)
	superpixelConfidence[i] = 0;

    if(point_cloud.isempty())
	return;
    const int imgwidth = panorama.Width();
    const int imgheight = panorama.Height();
    const Vector3d panCenter = panorama.GetCenter();
    const double depth_tolerence = 50;
    
    for(int ptid = 0; ptid<objectgroup.size(); ptid++){
	Vector3d curpt = point_cloud.GetPoint(objectgroup[ptid]).position;
	Vector3d offset = curpt - panCenter;
	Vector2d RGBpixel = panorama.Project(curpt);
	Vector3f curRGB = panorama.GetRGB(RGBpixel);
	if(curRGB.norm() < 0.00001)
	    continue;
	Vector2d depth_pixel = panorama.RGBToDepth(RGBpixel);
	double panoramadepth = panorama.GetDepth(depth_pixel);
//	double panoramadepth = depthmap.GetDepth(depth_pixel[0], depth_pixel[1]);
	double ptdepth = offset.norm();
	//visibility test
	if(ptdepth > panoramadepth + depth_tolerence)
	    continue;
	int superpixellabel = superpixel[(int)RGBpixel[1] * imgwidth + (int)RGBpixel[0]];
	superpixelConfidence[superpixellabel] += 1.0;
    }
    
    //perform erosion, to avoid conflicts on the border
    for(int iter=0; iter<erodeiter; iter++){
	for(const auto& curmap: pairmap){
	    pair<int,int> curpair = curmap.first;
	    if(curpair.first < ERODE_THRES || superpixelConfidence[curpair.second] < ERODE_THRES){
		superpixelConfidence[curpair.first] = 0;
		superpixelConfidence[curpair.second] = 0;
	    }
	}
	 
    }
}

void pairSuperpixel(const vector <int> &labels, int width, int height, map<pair<int,int>, int> &pairmap){
    //four connectivities
    for(int y=0;y<height-1;y++){
	for(int x=0;x<width-1;x++){
	    int label1 = labels[y*width+x]; //origin
	    int label2 = labels[(y+1)*width+x]; //down
	    int label3 = labels[y*width+x+1]; //right
	    int minlabelx = std::min(label1,label3);
	    int maxlabelx = std::max(label1,label3);
	    int minlabely = std::min(label1,label2);
	    int maxlabely = std::max(label1,label2);

	    if(label1 != label3){
		auto iter = pairmap.find(pair<int,int>(minlabelx,maxlabelx));
		if(iter != pairmap.end())
		    iter->second += 1;
		else
		    pairmap.insert(pair<pair<int,int>,int>(pair<int,int>(minlabelx,maxlabelx),1));
	    }

	    if(label1 != label2){
		auto iter = pairmap.find(pair<int,int>(minlabely,maxlabely));
		if(iter != pairmap.end())
		    iter->second += 1;
		else
		    pairmap.insert(pair<pair<int,int>,int>(pair<int,int>(minlabely,maxlabely),1));
	    }
	}
    }
}



void ReadObjectCloud(const FileIO &file_io, vector<PointCloud>&objectCloud, vector <vector< vector<int> > >&objectgroup){
    int roomid = 0;
    while(1){
	string filename = file_io.GetObjectPointClouds(roomid);
	string filename_wall = file_io.GetFloorWallPointClouds(roomid);

	ifstream fin(filename_wall.c_str());
	if(!fin.is_open())
	    break;
	fin.close();

	PointCloud curob, curwall;
	cout<< "Reading " << filename<<endl;
	curob.Init(filename);
	// cout<< "Reading " << filename_wall<<endl;
	// curwall.Init(filename_wall);
	// const vector<double> room_bbox = curwall.GetBoundingbox();
	
	//perform cleaning, remove floating objects and small objects
	Floorplan plan(file_io.GetFloorplan());
	const double floorheight = plan.GetFloorHeight(roomid);
	const double ceilingheight = plan.GetCeilingHeight(roomid);
	const double max_z = floorheight + (ceilingheight - floorheight) * 0.5;
//	const double max_z = room_bbox[4] + (room_bbox[5] - room_bbox[4]) * 0.5;
	const double min_points_num = 1000;
	const double min_points_size = 1e7;
	const Matrix3d transform_to_floorplan = plan.GetFloorplanToGlobal().transpose();
	
	vector <vector <int> >tempgroup;
	vector <int>points_to_remove;
	
	groupObject(curob, tempgroup);
	for(int objid=0; objid<tempgroup.size(); objid++){
	    bool isremove = false;
	    vector<double>object_bbox;
	    curob.GetObjectBoundingbox(objid, object_bbox);
	    double curvolume = curob.GetObjectBoundingboxVolume(objid);
	    if((curvolume < min_points_size) || (tempgroup[objid].size()<min_points_num))
		isremove = true;
	    Vector3d lowest(0.0,0.0,object_bbox[4]);
	    Vector3d lowest_floor = transform_to_floorplan * lowest;
	    if(lowest_floor[2] > max_z)
		isremove = true;
//	    if(object_bbox[4] > max_z)
//		isremove = true;

	    if(isremove){
		vector<int>points_to_remove;
		curob.GetObjectIndice(objid, points_to_remove);
		curob.RemovePoints(points_to_remove);
	    }
	}
//	cleanObjects(curob);
	vector <vector <int> > curgroup;
	groupObject(curob, curgroup);
	objectgroup.push_back(curgroup);
	objectCloud.push_back(curob);
	roomid++;
    }
}


void SetNeighbors(const std::vector<structured_indoor_modeling::Point>& points,
                  const int num_neighbors,
                  std::vector<std::vector<int> >* neighbors) {
     vector<float> point_data;
     {
	  point_data.reserve(3 * points.size());
	  for (int p = 0; p < points.size(); ++p) {
	       for (int i = 0; i < 3; ++i)
		    point_data.push_back(points[p].position[i]);
	  }
     }
     KDtree kdtree(point_data);
     vector<const float*> knn;
     vector<float> neighbor_distances(points.size());

     neighbors->clear();
     neighbors->resize(points.size());

     for (int p = 0; p < points.size(); ++p) {
	  knn.clear();
	  const Vector3f ref_point(points[p].position[0],
				   points[p].position[1],
				   points[p].position[2]);
                      
	  kdtree.find_k_closest_to_pt(knn, num_neighbors, &ref_point[0]);

	  for (int i = 0; i < (int)knn.size(); ++i) {
	       const int index = (knn[i] - &point_data[0]) / 3;
	       neighbors->at(p).push_back(index);
	  }
     }
}


void SmoothObjects(const std::vector<std::vector<int> >& neighbors,
                   std::vector<structured_indoor_modeling::Point>* points) {
     double unit = 0.0;
     int denom = 0;
     for (int p = 0; p < points->size(); ++p) {
	  for (int i = 0; i < neighbors[p].size(); ++i) {
	       unit += (points->at(p).position - points->at(neighbors[p][i]).position).norm();
	       ++denom;
	  }
     }
     unit /= denom;
     const double sigma = 2.0 * unit;

     // Smooth normals.
     vector<structured_indoor_modeling::Point> new_points = *points;
     for (int p = 0; p < points->size(); ++p) {
	  for (int i = 0; i < neighbors[p].size(); ++i) {
	       const int q = neighbors[p][i];
	       const double weight = exp(- (points->at(p).position - points->at(q).position).squaredNorm() / (2 * sigma * sigma));
	       new_points[p].normal += weight * points->at(q).normal;
	  }
	  if (new_points[p].normal != Vector3d(0, 0, 0))
	       new_points[p].normal.normalize();
     }
     *points = new_points;

     // Smooth positions.
     for (int p = 0; p < points->size(); ++p) {
	  double total_weight = 1.0;
	  for (int i = 0; i < neighbors[p].size(); ++i) {
	       const int q = neighbors[p][i];
	       // Estimate the position along the normal.
	       const Vector3d intersection = Intersect(points->at(p), points->at(q));
	       const double weight = exp(- (points->at(p).position - points->at(q).position).squaredNorm() / (2 * sigma * sigma));
	       new_points[p].position += weight * intersection;
	       total_weight += weight;
	  }
	  new_points[p].position /= total_weight;
     }
     *points = new_points;
}


double unaryDiffFunc(double confidence){
//     const double offset = 3.0;
//     const double maxv = 1.0;
    return max(gaussianFunc(confidence, 1.0),0.01);
}

double colorDiffFunc(int pix1,int pix2, const vector <Vector3d>&averageRGB){
    const double offset = 50;
    const double maxv = 1.0;
    Vector3d colordiff = averageRGB[pix1] - averageRGB[pix2];
//    return max(sigmaFunc(colordiff.norm(),offset,maxv,1.0),0.01);
    return max(gaussianFunc(colordiff.norm(), 15),0.1);
}


void MRFOptimizeLabels(const vector<int>&superpixelConfidence,  const map<pair<int,int>,int> &pairmap, const vector<Vector3d>&averageRGB, float smoothnessweight, vector <int> &superpixelLabel){
    int superpixelnum = superpixelConfidence.size();
    vector<MRF::CostVal>data(superpixelnum * 2);
    vector<MRF::CostVal>smooth(4);

    //model
    for(int i=0;i<superpixelnum;i++){
	data[2*i] = (MRF::CostVal)(gaussianFunc(1.0/((float)superpixelConfidence[i] + 0.001), 1) * 1000) ;    //assign 0
	data[2*i+1] = (MRF::CostVal)(gaussianFunc((float)superpixelConfidence[i], 1) * 1000);  //assign 1
    }
    smooth[0] = 0; smooth[3] = 0;
    smooth[1] = 1; smooth[2] = 1;

    DataCost *dataterm = new DataCost(&data[0]);
    SmoothnessCost *smoothnessterm = new SmoothnessCost(&smooth[0]);
    EnergyFunction *energy = new EnergyFunction(dataterm,smoothnessterm);

    MRF *mrf;
    mrf = new Expansion(superpixelnum, 2, energy);

    //solve
    mrf->initialize();

    for(auto mapiter:pairmap){
	pair<int,int> curpair = mapiter.first;
	//    MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)(unaryDiffFunc(curpair.first,curpair.second,superpixelConfidence) * 1000);
	MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)(colorDiffFunc(curpair.first,curpair.second,averageRGB) * 500);
	//    cout<<colorDiffFunc(curpair.first,curpair.second,averageRGB)<<endl;
	mrf->setNeighbors(curpair.first,curpair.second, weight);
    }
  
    mrf->clearAnswer();
  
    for(int i=0;i<superpixelnum;i++)
	mrf->setLabel(i,0);

    MRF::EnergyVal E;
    E = mrf->totalEnergy();

    printf("Energy at the Start= %d (%d,%d)\n",E,mrf->dataEnergy(),mrf->smoothnessEnergy());

    float t;
    mrf->optimize(100,t);
    E = mrf->totalEnergy();
    printf("Energy at the end = %d (%d,%d) (%g secs)\n",E,mrf->dataEnergy(),mrf->smoothnessEnergy(),t);
  
    //copy the solution
    superpixelLabel.clear();
    superpixelLabel.resize(superpixelnum);
    for(int i=0;i<superpixelnum;i++){
	superpixelLabel[i] = mrf->getLabel(i);
    }
    delete mrf;
    delete smoothnessterm;
    delete dataterm;
}


void MRFOptimizeLabels_multiLayer(const vector< vector<double> >&superpixelConfidence, const map<pair<int,int>,int> &pairmap, const vector< Vector3d > &averageRGB, float smoothweight, int numlabels, vector <int>& superpixelLabel){

    int superpixelnum = superpixelConfidence[0].size();

    vector<MRF::CostVal>data(superpixelnum * numlabels);
    vector<MRF::CostVal>smooth(numlabels * numlabels);

    for(int i=0;i<superpixelnum;i++){
	for(int label=0;label<numlabels;label++){
	    if(averageRGB[i].norm() < 0.001 && label < numlabels - 1){
		data[numlabels * i + label] = 10000;
		continue;
	    }
	    //	    data[numlabels * i + label] = (MRF::CostVal)( gaussianFunc((float)superpixelConfidence[label][i],1) * 1000);
	    data[numlabels * i + label] = (MRF::CostVal)(unaryDiffFunc(superpixelConfidence[label][i]) * 1000);
	}
    }

    for(int label1=0; label1<numlabels; label1++){
	for(int label2=0; label2<numlabels; label2++){
	    if(label1 == label2){
		smooth[label1 * numlabels + label2] = 0;
	    }
	    else{
		smooth[label1 * numlabels + label2] = 1;
		smooth[label2 * numlabels + label1] = 1;
	    }
	}
    }
  
    DataCost *dataterm = new DataCost(&data[0]);
    SmoothnessCost *smoothnessterm = new SmoothnessCost(&smooth[0]);
    EnergyFunction *energy = new EnergyFunction(dataterm,smoothnessterm);

    MRF *mrf;
    mrf = new Expansion(superpixelnum, numlabels, energy);

    //solve
    mrf->initialize();

 
    for(const auto &mapiter:pairmap){
	pair<int,int> curpair = mapiter.first;
	MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)(colorDiffFunc(curpair.first,curpair.second,averageRGB) * 1000 * smoothweight);
	mrf->setNeighbors(curpair.first,curpair.second, weight);
    }
  
    mrf->clearAnswer();

    for(int i=0;i<superpixelnum;i++)
	mrf->setLabel(i,0);

    MRF::EnergyVal E;
    E = mrf->totalEnergy();
    printf("Energy at the Start= %d (%d,%d)\n",E,mrf->dataEnergy(),mrf->smoothnessEnergy());

    float t;
    cout<<"solving..."<<endl;
    mrf->optimize(100,t);
    E = mrf->totalEnergy();
    printf("Energy at the end = %d (%d,%d) (%g secs)\n",E,mrf->dataEnergy(),mrf->smoothnessEnergy(),t);
  
    //copy the solution
    superpixelLabel.clear();
    superpixelLabel.resize(superpixelnum);
    for(int i=0;i<superpixelnum;i++){
	superpixelLabel[i] = mrf->getLabel(i);
    }
    delete mrf;
    delete smoothnessterm;
    delete dataterm;
}

void backProjectObject(const Panorama &panorama,const PointCloud& objectcloud, const vector< vector<int> >&objectgroup, const vector<int>&segmentation, const vector< vector<int> >&labelgroup, vector<list<PointCloud> >&objectlist, const int panoramaid, const int roomid){
    const int backgroundlabel = *max_element(segmentation.begin(),segmentation.end());
    const int imgwidth = panorama.Width();
    const int imgheight = panorama.Height();
    const int depthwidth = panorama.DepthWidth();
    const int depthheight = panorama.DepthHeight();
    const vector <double> bounding_box  = objectcloud.GetBoundingbox();
    const double max_z = bounding_box[4] + (bounding_box[5] - bounding_box[4]) * 0.7; //remove ceil points
    const double min_volume = 1e5;
    const double min_XYdensity = 0;

    vector< vector<structured_indoor_modeling::Point> >point_to_add(backgroundlabel);
    vector<PointCloud>pointcloud_to_merge(backgroundlabel);
    
    //get depth map for each object
    vector <DepthFilling> objectdepth(backgroundlabel);
    objectlist.resize(backgroundlabel);
    
    for(int objectid=0; objectid<backgroundlabel; objectid++){
    	objectdepth[objectid].Init(objectcloud, panorama, objectgroup[objectid], false);
	char buffer[100];
	sprintf(buffer, "depth/depth_object_pan%03d_object%03d.png", panoramaid, objectid);
	objectdepth[objectid].SaveDepthmap(string(buffer));
    	//mask for current object
    	for(int spix=0; spix<segmentation.size(); spix++){
    	    if(segmentation[spix] != objectid)
    		continue;
    	    for(int pixid=0; pixid<labelgroup[spix].size(); pixid++){
    		int pix = labelgroup[spix][pixid];
    		Vector2d RGBpixel((double)(pix%panorama.Width()), (double)(pix/panorama.Width()));
    		Vector2d depthpixel = panorama.RGBToDepth(RGBpixel);
    		objectdepth[segmentation[spix]].setMask((int)depthpixel[0],(int)depthpixel[1], true);
    	    }
    	}
    	objectdepth[objectid].fill_hole(panorama);
    }
  
    for(int superpixelid=0; superpixelid<segmentation.size(); superpixelid++){
    	if(segmentation[superpixelid] < backgroundlabel){   //object
    	    for(int pixelid=0; pixelid<labelgroup[superpixelid].size(); pixelid++){
    		int pix = labelgroup[superpixelid][pixelid];
    		Vector2d pixloc((double)(pix % imgwidth), (double)(pix / imgwidth));
    		Vector2d depthloc = panorama.RGBToDepth(pixloc);
    		Vector3f curcolor = panorama.GetRGB(pixloc);
    		float temp = curcolor[2];
    		curcolor[2] = curcolor[0];
    		curcolor[0] = temp;
    		double depthv = objectdepth[segmentation[superpixelid]].GetDepth(depthloc[0],depthloc[1]);
    		if(curcolor.norm() == 0 || depthv < 0)
    		    continue;

    		Vector3d worldcoord = panorama.Unproject(pixloc, depthv);
    		if(worldcoord[2] >= max_z)
    		    continue;
		
    		structured_indoor_modeling::Point curpt;
    		curpt.position = worldcoord;
    		curpt.color = curcolor;
    		curpt.depth_position = Vector2i(0,0);
    		curpt.normal = Vector3d(0,0,0);
    		curpt.intensity = 0.0;
    		curpt.object_id = segmentation[superpixelid];
    		point_to_add[segmentation[superpixelid]].push_back(curpt);
    	    }
    	}
    }

    for(int objid=0; objid<pointcloud_to_merge.size(); objid++){
	pointcloud_to_merge[objid].AddPoints(point_to_add[objid]);
	//ignore small object
	if(pointcloud_to_merge[objid].GetBoundingboxVolume() < min_volume)
	    continue;

	if(objectlist[objid].empty()){
	    objectlist[objid].push_back(pointcloud_to_merge[objid]);
	    continue;
	}
	if(pointcloud_to_merge[objid].GetNumPoints() > objectlist[objid].front().GetNumPoints())
	    objectlist[objid].push_front(pointcloud_to_merge[objid]);
	else
	    objectlist[objid].push_back(pointcloud_to_merge[objid]);

	//add input point cloud
    }
}

double evalError(vector<Vector3f>&src, vector<Vector3f>&dst, const Matrix3f &transform){
    double err = 0.0;
    for(int i=0; i<src.size(); i++)
	err += (dst[i] - transform*src[i]).norm();
    return err;
}

void colorTransform_RANSAC(vector<Vector3f>&src, vector<Vector3f>&dst, Matrix3f &transform, const int maxiter){
    const double max_allowed_err = 10.0;
    const int max_size = src.size();
    Matrix3f temptrans;
    double min_err = numeric_limits<double>::max();
    int i;
    for(i=0; i<maxiter; i++){
	vector<Vector3f>src_ran;
	vector<Vector3f>dst_ran;
	vector<int>ran;
	srand((unsigned)time(NULL));
	for(int j=0; j<3; j++){
	    int id = rand()%max_size;
	    src_ran.push_back(src[id]);
	    dst_ran.push_back(dst[id]);
	}
	computeColorTransform(src_ran, dst_ran, temptrans);
	double curerr = evalError(src, dst, temptrans);
	if(curerr < min_err){
	    min_err = curerr;
	    transform = temptrans;
	}
	if(curerr < max_allowed_err)
	    break;
    }
    cout<<"RANSAC complete, iteration:" << i<<endl;
}

void computeColorTransform(vector<Vector3f>&src, vector<Vector3f>&dst, Matrix3f &transform){
    Matrix<float, Dynamic ,9>A;
    Matrix<float, Dynamic, 1>B;
    A = Matrix<float, Dynamic, 9>::Zero((int)src.size() * 3, 9);
    B = Matrix<float, Dynamic, 1>::Zero((int)src.size() * 3, 1);
    for(int i=0; i<src.size(); i++){
	A(i*3,0) = src[i][0];
	A(i*3,1) = src[i][1];
	A(i*3,2) = src[i][2];
	A(i*3+1,3) = src[i][0];
	A(i*3+1,4) = src[i][1];
	A(i*3+1,5) = src[i][2];
	A(i*3+2,6) = src[i][0];
	A(i*3+2,7) = src[i][1];
	A(i*3+2,8) = src[i][2];
	B(i*3,0) = dst[i][0];
	B(i*3+1,0) = dst[i][1];
	B(i*3+2,0) = dst[i][2];
    }
    Matrix<float,9,9>L = A.transpose() * A;
    Matrix<float,9,1>R = A.transpose() * B;

    Matrix<float,9,1>X = L.householderQr().solve(R);
    
    for(int y=0;y<3;y++){
	for(int x=0;x<3;x++)
	    transform(y,x) = X(y*3+x,0);
    }
}
//Merge object in objetlist, compensate for exposure difference
void mergeObject(vector<vector<list<PointCloud> > >&objectlist, const vector<PointCloud> &objectcloud, vector<PointCloud> &resultcloud){
    const int grid_length = 500;
    const double max_conflict_ratio = 0.9;
    const double min_object_volume = 1e7;
    const int min_object_points_num = 1500;
    
    resultcloud.clear();
    resultcloud.resize(objectlist.size());

    vector<vector<vector<int> > >grid(grid_length);
        
    for(auto &v: grid){
    	v.resize(grid_length);
    	for(auto &u: v)
    	    u.resize(grid_length);
    }
    
    for(int roomid = 0; roomid<objectlist.size(); roomid++){
	for(int objid = 0; objid<objectlist[roomid].size(); objid++){
	    cout<<"room "<<roomid<<' '<<"objid "<<objid<<endl;
	    auto firstpart = objectlist[roomid][objid].begin();
	    auto iter = objectlist[roomid][objid].begin();

	    if(iter == objectlist[roomid][objid].end())
		continue;

	    //voxel grid approach
	    //init voxel grid
	    for(auto &v:grid){
		for(auto &u: v){
		    for(auto &w: u)
			w = -1;
		}
	    }
	    vector<double>bbox;
	    objectcloud[roomid].GetObjectBoundingbox(objid, bbox);
	    double unitlength = 10.0;

	    ++iter;  //begin from the second part

	    for(; iter!=objectlist[roomid][objid].end();++iter){
		if(iter->GetNumPoints() < 100)
		    continue;

		vector<Vector3f>conflictRGB_ori;
		vector<Vector3f>conflictRGB_cur;
		
		int diff_count = 0;

		for(int ptid=0; ptid<firstpart->GetNumPoints(); ptid++){
		    int x = ((*firstpart).GetPoint(ptid).position[0] - bbox[0]) / unitlength;
		    int y = ((*firstpart).GetPoint(ptid).position[1] - bbox[2]) / unitlength;
		    int z = ((*firstpart).GetPoint(ptid).position[2] - bbox[4]) / unitlength;
		    if(x<0 || x>=grid_length || y<0 || y>=grid_length || z<0 || z>=grid_length)
			continue;
		    grid[x][y][z] = ptid;
		}

		vector<int>point_to_remove;
		for(int ptid=0; ptid<iter->GetNumPoints(); ptid++){
		    //remove points outside the origin boundingbox
		    if((*iter).GetPoint(ptid).position[0]<bbox[0] || (*iter).GetPoint(ptid).position[0]>bbox[1]){
			point_to_remove.push_back(ptid);
			continue;			
		    }
		    if((*iter).GetPoint(ptid).position[1]<bbox[2] || (*iter).GetPoint(ptid).position[1]>bbox[3]){
			point_to_remove.push_back(ptid);
			continue;
		    }
		    if((*iter).GetPoint(ptid).position[2]<bbox[4] || (*iter).GetPoint(ptid).position[2]>bbox[5]){
			point_to_remove.push_back(ptid);
			continue;
		    }
		    int x = ((*iter).GetPoint(ptid).position[0] - bbox[0]) / unitlength;
		    int y = ((*iter).GetPoint(ptid).position[1] - bbox[2]) / unitlength;
		    int z = ((*iter).GetPoint(ptid).position[2] - bbox[4]) / unitlength;
		    if(x<0 || x>=grid_length || y<0 || y>=grid_length || z<0 || z>=grid_length){
			point_to_remove.push_back(ptid);
			continue;
		    }
		    
		    if(grid[x][y][z] != -1){
			Vector3f oricolor = (*firstpart).GetPoint(grid[x][y][z]).color;
			Vector3f curcolor = (*iter).GetPoint(ptid).color;
			conflictRGB_ori.push_back(oricolor);
			conflictRGB_cur.push_back(curcolor);
			diff_count++;
			point_to_remove.push_back(ptid);
		    }
		}
		double cur_conflict_ratio = (double)diff_count / (double)(*iter).GetNumPoints();
		if(cur_conflict_ratio > max_conflict_ratio)
		    continue;
		cout<<"Conflict count: "<<diff_count<<endl;
		(*iter).RemovePoints(point_to_remove);

		//compute Color transform
		Matrix3f transform;
		computeColorTransform(conflictRGB_cur, conflictRGB_ori, transform);
		
		for(int ptid=0; ptid<iter->GetNumPoints(); ptid++){
		    Vector3f curcolor = iter->GetPoint(ptid).color;
		    Vector3f newcolor = transform * curcolor;
		    if(newcolor[0]<0 || newcolor[1] >255 || newcolor[1]<0 || newcolor[1]>255 || newcolor[2]<0 || newcolor[2]>255)
			continue;
		    iter->SetColor(ptid, transform * curcolor);
		}
		objectlist[roomid][objid].front().AddPoints(*iter, true);
	    }

	    double object_volume = objectlist[roomid][objid].front().GetBoundingboxVolume();
	    int object_point_num = objectlist[roomid][objid].front().GetNumPoints();
	    if(object_volume > min_object_volume && object_point_num > min_object_points_num)
		resultcloud[roomid].AddPoints(objectlist[roomid][objid].front(), true);
	}
    }
}

//remove small objects, re-assign object id
void cleanObjects(PointCloud &pc){
    //reassign object_id
    vector<int>objectid;
    vector<bool>hash(pc.GetNumObjects());
    for(int i=0; i<hash.size(); i++)
	hash[i] = false;
    for(int ptid=0; ptid<pc.GetNumPoints(); ptid++){
	int curid = pc.GetPoint(ptid).object_id;
	if(!hash[curid]){
	    hash[curid] = true;
	    objectid.push_back(curid);
	}
    }
    
    for(int ptid=0; ptid<pc.GetNumPoints(); ptid++){
	for(int i=0;i<objectid.size(); i++){
	    if(pc.GetPoint(ptid).object_id == objectid[i])
		pc.GetPoint(ptid).object_id = i;
	}
    }
    pc.Update();
}

void ICP(PointCloud &src, const PointCloud &tgt, const int num_iter, const int downsample){
    cout<<"ICP..."<<endl;
    const int srcnum = src.GetNumPoints();
    const float max_dist = 100;
     
    //building kd-tree for target
//     cout<<"Building KD tree"<<endl;
    flann::KDTreeIndexParams indexParams(4);

    Mat featurepoints(tgt.GetNumPoints(), 3, CV_32F);
    Mat query(srcnum/downsample,3,CV_32F);
    for(int i=0; i<tgt.GetNumPoints(); i+=downsample){
	structured_indoor_modeling::Point curpt = tgt.GetPoint(i);
	featurepoints.at<float>(i/downsample,0) = (float)curpt.position[0];
	featurepoints.at<float>(i/downsample,1) = (float)curpt.position[1];
	featurepoints.at<float>(i/downsample,2) = (float)curpt.position[2];

    }
    flann::Index searchtree(featurepoints, indexParams);

    typedef Matrix<double, 3,Dynamic> Matrix3;

    for(int iter=1; iter<=num_iter; iter++){
//	  cout<<"---------------"<<endl;
//	  cout<<"Iteration "<<iter<<endl;
	  
	for(int i=0; i<srcnum; i+=downsample){
	    query.at<float>(i/downsample,0) = (float)src.GetPoint(i).position[0];
	    query.at<float>(i/downsample,1) = (float)src.GetPoint(i).position[1];
	    query.at<float>(i/downsample,2) = (float)src.GetPoint(i).position[2];
	}
	  
	Mat searchres(srcnum/downsample, 1, CV_32S, Scalar::all(-1));
	Mat dists(srcnum/downsample, 1, CV_32F);

	searchtree.knnSearch(query, searchres, dists, 1, flann::SearchParams(32));
	  
	Vector3d center_tgt(0,0,0);
	Vector3d center_src = src.GetCenter();

	int count = 0;
	for(int i=0; i<srcnum/downsample; i++){
//	      if(dists.at<float>(0,i) > max_dist)
//		  continue;
	    structured_indoor_modeling::Point curpt = tgt.GetPoint(searchres.at<int>(0,i));
	    center_tgt += curpt.position;
	    count++;
	}
	if(count > 0)
	    center_tgt /= (double)count;
	else
	    return;

	// for(int i=0; i<srcnum; i+=downsample){
	//      P(0,i/downsample) = src.GetPoint(i).position[0] - center_src[0];
	//      P(1,i/downsample) = src.GetPoint(i).position[1] - center_src[1];
	//      P(2,i/downsample) = src.GetPoint(i).position[2] - center_src[2];

	//      Q(0,i/downsample) = tgt.GetPoint(searchres.at<int>(0,i/downsample)).position[0] - center_tgt[0];
	//      Q(1,i/downsample) = tgt.GetPoint(searchres.at<int>(0,i/downsample)).position[1] - center_tgt[1];
	//      Q(2,i/downsample) = tgt.GetPoint(searchres.at<int>(0,i/downsample)).position[2] - center_tgt[2];
	// }
	// M = P * Q.transpose();
	// JacobiSVD<Matrix<double,3,3>>svd(M,ComputeFullU|ComputeFullV);
	// R = svd.matrixV().transpose() * svd.matrixU();

//	  cout<<"Translation:"<<endl;
	Vector3d trans = center_tgt - center_src;
//	  cout<<trans.transpose()<<endl;
//	  cout<<"Rotation:"<<endl;
//	  cout<<R<<endl;
	src.Translate(center_tgt - center_src);
//	  src.Rotate(R);
//	  src.Translate(center_tgt);

    } //iteration
    cout<<"ICP done"<<endl;
}

void radiusRemovalFilter(PointCloud &pc, const double radius, const int min_count){
    
}


