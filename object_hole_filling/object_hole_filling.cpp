#include "object_hole_filling.h"
#include "SLIC.h"
#include <numeric>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <fstream>
#include <Eigen/SVD>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

#define PI 3.1415927
#define COLORTABLE_LENGTH 7
#define ERODE_THRES 0
//Vec3b colortable[] = {Vec3b(255,0,0), Vec3b(0,255,0), Vec3b(0,0,255), Vec3b(255,255,0), Vec3b(255,0,255), Vec3b(0,255,255), Vec3b(128,0,0), Vec3b(0,128,0), Vec3b(0,0,128), Vec3b(128,128,0), Vec3b(128,0,128), Vec3b(0,128,128), Vec3b(255,128,128),Vec3b(128,255,128),Vec3b(128,128,255),Vec3b(128,128,128)};

Vec3b colortable[] = {Vec3b(255,0,0), Vec3b(0,255,0), Vec3b(0,0,255), Vec3b(255,255,0), Vec3b(255,0,255), Vec3b(0,255,255),Vec3b(255,255,255)};

void initPanorama(const FileIO &file_io, vector<Panorama>&panorama, vector< vector<int> >&labels, const int expected_num, vector<int>&numlabels, vector<DepthFilling>&depth, int &imgwidth, int &imgheight, const int startid, const int endid){
    cout<<"Init panorama..."<<endl;

    char buffer[100];
    
    panorama.resize(endid - startid + 1);
    labels.resize(endid - startid + 1);
    depth.resize(endid - startid + 1);
    numlabels.resize(endid - startid + 1);
    
    for(int id=startid; id<=endid; id++){
	int curid = id - startid;
	cout<<"Panorama "<<id<<endl;
	panorama[curid].Init(file_io, id);
	
	PointCloud curpc;
	cout<<"reading pnaorama  point cloud..."<<endl;
	curpc.Init(file_io, id);
	curpc.ToGlobal(file_io, id);
    
	//Get depthmap
	cout<<"Processing depth map..."<<endl;
	sprintf(buffer,"depth/panorama%03d.depth",id);
	if(!depth[curid].ReadDepthFromFile(string(buffer))){
	    depth[curid].Init(curpc, panorama[curid]);
	    depth[curid].fill_hole(panorama[curid]);
	    depth[curid].SaveDepthFile(string(buffer));
	}
	sprintf(buffer,"depth/panoramaDepth%03d.png",id);
	depth[curid].SaveDepthmap(string(buffer));
	
	imgwidth = panorama[curid].Width();
	imgheight = panorama[curid].Height();
	
	labels[curid].resize(imgwidth*imgheight);

	sprintf(buffer,"superpixel/SLIC%03d",id);
	ifstream labelin(buffer, ios::binary);
	if(!labelin.is_open()){
	    cout<<"Performing SLICO Superpixel..."<<endl;
	    Mat pan = panorama[curid].GetRGBImage().clone();
	    SLIC slic;
	    vector<unsigned int>imagebuffer;
	    MatToImagebuffer(pan, imagebuffer);
	    slic.PerformSLICO_ForGivenK(&imagebuffer[0],imgwidth,imgheight,&labels[curid][0],numlabels[curid],expected_num,0.0);
	    slic.DrawContoursAroundSegmentsTwoColors(&imagebuffer[0],&labels[curid][0],imgwidth,imgheight);
	    
	    sprintf(buffer,"superpixel/SLIC%03d",id);
	    slic.SaveSuperpixelLabels(&labels[curid][0],imgwidth,imgheight,numlabels[curid]," ",string(buffer));
	    cout<<"numlabels: "<<numlabels[curid]<<endl;
	    Mat out;
	    ImagebufferToMat(imagebuffer, imgwidth, imgheight, out);
	    sprintf(buffer,"superpixel/SLIC%03d.png",id); 
	    imwrite(buffer,out);
	    waitKey(10);
	}else{
	    cout <<"Reading superpixel from file"<<endl;
	    labelin.read((char*)&numlabels[curid], sizeof(int));
	    for(int i=0;i<labels[curid].size();i++){
		labelin.read((char*)&labels[curid][i], sizeof(int));
	    }
	    labelin.close();
	}
	cout<<endl;
    }
}


void MatToImagebuffer(const Mat image, vector<unsigned int>&imagebuffer){
    if(!image.data){
	cout << "invlid image"<<endl;
	exit(-1);
    }
    int imgheight = image.rows;
    int imgwidth = image.cols;
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


    for(int groupid = 0;groupid<superpixelConfidence.size();groupid++){
	 Mat outmask(imgheight,imgwidth,CV_8UC3, Scalar(0,0,0));
	 int colorid = groupid % COLORTABLE_LENGTH;
	for(int i=0;i<imgwidth*imgheight;++i){
	    int x = i % imgwidth;
	    int y = i / imgwidth;
///	    double curconfidence =(double) (superpixelConfidence[groupid][labels[i]] - minc) / (double)(maxc - minc);
	    double curconfidence = (double)superpixelConfidence[groupid][labels[i]];
	    // double r = (double)colortable[colorid][0] * curconfidence * 4;
	    // double g = (double)colortable[colorid][1] * curconfidence * 4;
	    // double b = (double)colortable[colorid][2] * curconfidence * 4;
	    double r = (double)curconfidence * 4;
	    double g = (double)curconfidence * 4;
	    double b = (double)curconfidence * 4;
	    Vec3b curpix((uchar)r, (uchar)g, (uchar)b);
	    outmask.at<Vec3b>(y,x) += curpix;
	}
	sprintf(buffer,"object_project/objectmask_panorama%03d_room%03d_group%03d.jpg",id, roomid, groupid);
	imwrite(buffer, outmask);
	waitKey(10);

    }
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
    sprintf(buffer,"object_project/optimize_pan%03d_room%03d.png",id,roomid);
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


int groupObject(const PointCloud &point_cloud, vector <vector<int> >&objectgroup, vector<double>&objectVolume){

    objectgroup.resize(point_cloud.GetNumObjects());
    objectVolume.resize(point_cloud.GetNumObjects());

    vector <vector <double> >boundingbox(objectgroup.size());
    for(int i=0;i<boundingbox.size();i++){
	boundingbox[i].resize(6);
	boundingbox[i][0] = 1e100; boundingbox[i][1] = -1e100;
	boundingbox[i][2] = 1e100; boundingbox[i][3] = -1e100;
	boundingbox[i][4] = 1e100; boundingbox[i][5] = -1e100;
    }
    for(int i=0;i<point_cloud.GetNumPoints();++i){
	structured_indoor_modeling::Point curpt = point_cloud.GetPoint(i);
	int curid = curpt.object_id;
	boundingbox[curid][0] = min(boundingbox[curid][0],curpt.position[0]);
	boundingbox[curid][1] = max(boundingbox[curid][1],curpt.position[0]);
	boundingbox[curid][2] = min(boundingbox[curid][2],curpt.position[1]);
	boundingbox[curid][3] = max(boundingbox[curid][3],curpt.position[1]);
	boundingbox[curid][4] = min(boundingbox[curid][4],curpt.position[2]);
	boundingbox[curid][5] = max(boundingbox[curid][5],curpt.position[2]);
	objectgroup[curid].push_back(i);
    }
    for(int i=0;i<boundingbox.size();i++){
	objectVolume[i] = (boundingbox[i][1] - boundingbox[i][0])*(boundingbox[i][3] - boundingbox[i][2]) * (boundingbox[i][5] - boundingbox[i][4]);
    }
  
    return (int)objectgroup.size();
}



void getSuperpixelConfidence(const PointCloud &point_cloud,const vector<int> &objectgroup,  const Panorama &panorama, const DepthFilling& depthmap, const vector<int> &superpixel,const vector< vector<int> >&labelgroup, const map<pair<int,int>,int>&pairmap,  vector <double> &superpixelConfidence, const int superpixelnum, const int erodeiter){
     if(superpixelConfidence.size() > 0)
	  superpixelConfidence.clear();
    superpixelConfidence.resize(superpixelnum);
    
    for(int i=0;i<superpixelConfidence.size();++i)
	superpixelConfidence[i] = 0;

    if(point_cloud.isempty())
	return;
    int imgwidth = panorama.Width();
    int imgheight = panorama.Height();

    for(int ptid = 0; ptid<objectgroup.size(); ptid++){
	Vector3d curpt = point_cloud.GetPoint(objectgroup[ptid]).position;
	Vector3d panCenter = panorama.GetCenter();
	Vector3d offset = curpt - panCenter;
	Vector2d RGBpixel = panorama.Project(curpt);
	Vector2d depth_pixel = panorama.RGBToDepth(RGBpixel);
	double depthv = depthmap.GetDepth(depth_pixel[0],depth_pixel[1]);
	double curdepth = offset.norm();
	//visibility test
	if(curdepth > depthv)
	    continue;
	int superpixellabel = superpixel[(int)RGBpixel[1] * imgwidth + (int)RGBpixel[0]];
	superpixelConfidence[superpixellabel] += 1.0;
    }
    //perform erosion, to avoid conflicts on the border
    for(int iter=0; iter<erodeiter; iter++){
	 for(const auto& curmap: pairmap){
	      pair<int,int> curpair = curmap.first;
	      if(curpair.first > ERODE_THRES && superpixelConfidence[curpair.second] < ERODE_THRES){
		   superpixelConfidence[curpair.first] = 0;
	      }
	      if(curpair.second > ERODE_THRES && superpixelConfidence[curpair.first] < ERODE_THRES){
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



void ReadObjectCloud(const FileIO &file_io, vector<PointCloud>&objectCloud, vector <vector< vector<int> > >&objectgroup, vector <vector <double> >&objectVolume){
     int roomid = 5;
    while(1){
	string filename = file_io.GetObjectPointClouds(roomid);
	string filename_wall = file_io.GetFloorWallPointClouds(roomid++);

	ifstream fin(filename_wall.c_str());
	if(!fin.is_open())
	    break;
	fin.close();

	PointCloud curob, curwall;
	cout<< "Reading " << filename<<endl;
	curob.Init(filename);
	cout<< "Reading " << filename_wall<<endl;
	curwall.Init(filename_wall);
	for(int i=0;i<curwall.GetNumPoints();i++){
	    curwall.GetPoint(i).object_id = 0 ;
	}
	curob.AddPoints(curwall);

	objectCloud.push_back(curob);
	vector <vector <int> > curgroup;
	vector <double> curvolume;
	groupObject(curob, curgroup, curvolume);
	objectgroup.push_back(curgroup);
	objectVolume.push_back(curvolume);
	break;
    }
}


double unaryDiffFunc(double confidence){
     const double offset = 3.0;
     const double maxv = 1.0;
     return max(sigmaFunc(confidence, offset, maxv, 1.0), 0.1);
}

double colorDiffFunc(int pix1,int pix2, const vector <Vector3d>&averageRGB){
    const double offset = 80;
    const double maxv = 1.0;
    Vector3d colordiff = averageRGB[pix1] - averageRGB[pix2];
    return max(sigmaFunc(colordiff.norm(),offset,maxv,1.0),0.1);
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

#if 0
    //statistics for colordiff
    vector <double> colordiffarray;
    for(const auto &mapiter:pairmap){
	pair<int,int> curpair = mapiter.first;
	double v = (double)mapiter.second * std::abs((averageRGB[curpair.first] - averageRGB[curpair.second]).norm());
	colordiffarray.push_back(v);
    }

    double mean = std::accumulate(colordiffarray.begin(), colordiffarray.end(), 0.0) / (double)colordiffarray.size();
    vector<double>diff(colordiffarray.size());
    std::transform(colordiffarray.begin(),colordiffarray.end(),diff.begin(),std::bind2nd(std::minus<double>(),mean));
    double var = std::sqrt(std::inner_product(diff.begin(),diff.end(),diff.begin(),0.0) / ((double)diff.size() - 1));
    cout<<"size of array: "<<colordiffarray.size()<<' '<<"mean: "<<mean<<' '<<"variance: "<<var<<endl;
    ofstream mystream("color.txt");
    std::copy(colordiffarray.begin(),colordiffarray.end(),std::ostream_iterator<double>(mystream,"\n"));
    mystream.close();
#endif
 
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



void mergeObject(const Panorama &panorama, const DepthFilling& depth,const PointCloud& objectcloud, const vector< vector<int> >&objectgroup, const vector<int>&segmentation, const vector< vector<int> >&labelgroup, vector<list<PointCloud> >&objectlist){
    const int backgroundlabel = *max_element(segmentation.begin(),segmentation.end());
    const int imgwidth = panorama.Width();
    const int imgheight = panorama.Height();
    const int depthwidth = panorama.DepthWidth();
    const int depthheight = panorama.DepthHeight();
    const vector <double> bounding_box  = objectcloud.GetBoundingbox();
    const double max_z = bounding_box[4] + (bounding_box[5] - bounding_box[4]) * 0.8; //remove ceil points
    const double min_volume = 1e5;
    const double min_XYdensity = 0;

    vector< vector<structured_indoor_modeling::Point> >point_to_add(backgroundlabel);
    vector<PointCloud>pointcloud_to_merge(backgroundlabel);
    //get depth map for each object
    vector <DepthFilling> objectdepth(backgroundlabel);

    objectlist.resize(backgroundlabel);

    for(int objectid=0; objectid<backgroundlabel; objectid++){
    	objectdepth[objectid].Init(objectcloud, panorama, objectgroup[objectid], false);
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
	vector<double>curbbox = pointcloud_to_merge[objid].GetBoundingbox();
	//if the point is sparse at XY plane, abort
	// double areaXY = (curbbox[1] - curbbox[0]) * (curbbox[3] - curbbox[2]);
	// if(areaXY < min_XYdensity)
	//     continue;

//	perform ICP, align all parts to original segmentation cloud
	PointCloud curobj_ori;
	vector<structured_indoor_modeling::Point>curpt_to_add;
	for(const auto&v :objectgroup[objid]){
	    curpt_to_add.push_back(objectcloud.GetPoint(v));
	}
	curobj_ori.AddPoints(curpt_to_add);
	ICP(pointcloud_to_merge[objid], curobj_ori);

	if(objectlist[objid].empty()){
	    objectlist[objid].push_back(pointcloud_to_merge[objid]);
	    continue;
	}
	if(pointcloud_to_merge[objid].GetNumPoints() > objectlist[objid].front().GetNumPoints())
	    objectlist[objid].push_front(pointcloud_to_merge[objid]);
	else
	    objectlist[objid].push_back(pointcloud_to_merge[objid]);
    }
}

//Merge object in objetlist, compensate for exposure difference
void backProjectObject(vector<vector<list<PointCloud> > >&objectlist, const vector<PointCloud> &objectcloud, vector<PointCloud> &resultcloud){
    const int grid_length = 200;
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
	    //////////////////////////////////////////
	    //TODO: compensate for exposure
	    //////////////////////////////////////////
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
	    double unitlength = -1;
	    unitlength = std::max(bbox[1]-bbox[0], unitlength);
	    unitlength = std::max(bbox[3]-bbox[2], unitlength);
	    unitlength = std::max(bbox[5]-bbox[4], unitlength);
	    unitlength /= (double)grid_length;
	    
	    if(objectlist[roomid][objid].size() < 2)
		continue;
	    
	    auto iter = objectlist[roomid][objid].begin();
	    for(int ptid=0; ptid<iter->GetNumPoints(); ptid++){
		int x = ((*iter).GetPoint(ptid).position[0] - bbox[0]) / unitlength;
		int y = ((*iter).GetPoint(ptid).position[1] - bbox[2]) / unitlength;
		int z = ((*iter).GetPoint(ptid).position[2] - bbox[4]) / unitlength;
		if(x<0 || x>grid_length || y<0 || y>=grid_length || z<0 || z>=grid_length)
		    continue;
		if(grid[x][y][z] == -1)
		    grid[x][y][z] = ptid;
	    }

	    ++iter;  //begin from the second part
	    for(; iter!=objectlist[roomid][objid].end(); ++iter){
		vector<int>point_to_remove;
		for(int ptid=0; ptid<iter->GetNumPoints(); ptid++){
		    int x = ((*iter).GetPoint(ptid).position[0] - bbox[0]) / unitlength;
		    int y = ((*iter).GetPoint(ptid).position[1] - bbox[2]) / unitlength;
		    int z = ((*iter).GetPoint(ptid).position[2] - bbox[4]) / unitlength;
		    if(x<0 || x>grid_length || y<0 || y>=grid_length || z<0 || z>=grid_length)
			continue;
		    if(grid[x][y][z] == -1)
			point_to_remove.push_back(ptid);
		}
		iter->RemovePoints(point_to_remove);
		objectlist[roomid][objid].front().AddPoints(*iter, true);
		objectlist[roomid][objid].erase(iter);
	    }
	    resultcloud[roomid].AddPoints(objectlist[roomid][objid].front(), true);
	}
    }
}

//remove small objects, re-assign object id
void cleanObjects(PointCloud &pc, const double min_volume){
    //clean small objects
    for(int objid = 0; objid < pc.GetNumObjects(); objid++){
	double curvolume = pc.GetObjectBoundingboxVolume(objid);
	if(curvolume >= min_volume)
	    continue;
	vector<int>points_to_remove;
	pc.GetObjectIndice(objid, points_to_remove);
	pc.RemovePoints(points_to_remove);
    }
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


//Merge algorithm
//Given a point, remove all points that are inside the ball around the point.
void mergeVertices(PointCloud &pc, int resolution){

     if(pc.GetNumPoints() == 0 )
	  return;

     //automatically compute radius
     vector<double>boundingbox  = pc.GetBoundingbox();
     vector<double>diff(3);
     diff[0] = abs(boundingbox[1]-boundingbox[0]);
     diff[1] = abs(boundingbox[3]-boundingbox[2]);
     diff[2] = abs(boundingbox[5]-boundingbox[4]);
     double maxdiff = *max_element(diff.begin(),diff.end());
     double radius = maxdiff / static_cast<double>(resolution);   //two vertices per pixel
     cout<<"radius: "<<radius<<endl;
     
     int unit = pc.GetNumPoints() / 100;
     flann::KDTreeIndexParams indexParams(5);

     Mat featurepoints(pc.GetNumPoints(),3,CV_32F);
     Mat query(1,3,CV_32F);
    
     //build index
     int count=0;
     for(int i=0;i<pc.GetNumPoints();i++){
	  structured_indoor_modeling::Point curpt = pc.GetPoint(i);
	  featurepoints.at<float>(count,0) = (float)curpt.position[0];
	  featurepoints.at<float>(count,1) = (float)curpt.position[1];
	  featurepoints.at<float>(count,2) = (float)curpt.position[2];
	  count++;
     }
     flann::Index searchtree(featurepoints,indexParams);

     vector<bool>mask(pc.GetNumPoints());
     vector<int>points_to_remove;
     for(int i=0;i<mask.size();i++)
	 mask[i] = true;
     
     for(int curcenter=0;curcenter<pc.GetNumPoints();curcenter++){
	  if(!mask[curcenter])
	       continue;
	  
	  if(curcenter % unit == 0)
	       cout<<curcenter / unit<<' ';

	  query.at<float>(0,0) = (float)pc.GetPoint(curcenter).position[0];
	  query.at<float>(0,1) = (float)pc.GetPoint(curcenter).position[1];
	  query.at<float>(0,2) = (float)pc.GetPoint(curcenter).position[2];

	  Mat searchres(1,pc.GetNumPoints(),CV_32S,Scalar::all(-1));
	  Mat dists(1,pc.GetNumPoints(),CV_32F);

	  searchtree.radiusSearch(query,searchres,dists,radius,pc.GetNumPoints(),flann::SearchParams(16));
	  for(int i=0;i<pc.GetNumPoints(); i++){
	       int curind = searchres.at<int>(0,i);
	       if(curind < 0)
		    break;
	       if(curind == curcenter)
		    continue;
	       mask[curind] = false;
	       points_to_remove.push_back(curind);
	  }
     }

     pc.RemovePoints(points_to_remove);

     cout<<endl;
}


void ICP(PointCloud &src, const PointCloud &tgt, const int num_iter){
     cout<<"ICP..."<<endl;
     const int srcnum = src.GetNumPoints();
     const float max_dist = 100;
     
     //building kd-tree for target
//     cout<<"Building KD tree"<<endl;
     flann::KDTreeIndexParams indexParams(5);

     Mat featurepoints(tgt.GetNumPoints(), 3, CV_32F);
     Mat query(srcnum,3,CV_32F);
     for(int i=0; i<tgt.GetNumPoints(); i++){
	  structured_indoor_modeling::Point curpt = tgt.GetPoint(i);
	  featurepoints.at<float>(i,0) = (float)curpt.position[0];
	  featurepoints.at<float>(i,1) = (float)curpt.position[1];
	  featurepoints.at<float>(i,2) = (float)curpt.position[2];

     }
     flann::Index searchtree(featurepoints, indexParams);

     typedef Matrix<double, 3,Dynamic> Matrix3;

     Matrix3 P = Matrix3::Zero(3,srcnum);
     Matrix3 Q = Matrix3::Zero(3,srcnum);
     Matrix<double,3,3> M, R;
     Matrix4d T;

     for(int iter=1; iter<=num_iter; iter++){
//	  cout<<"---------------"<<endl;
//	  cout<<"Iteration "<<iter<<endl;
	  
	  for(int i=0; i<srcnum; i++){
	       query.at<float>(i,0) = (float)src.GetPoint(i).position[0];
	       query.at<float>(i,1) = (float)src.GetPoint(i).position[1];
	       query.at<float>(i,2) = (float)src.GetPoint(i).position[2];
	  }
	  
	  Mat searchres(srcnum, 1, CV_32S, Scalar::all(-1));
	  Mat dists(srcnum, 1, CV_32F);

	  searchtree.knnSearch(query, searchres, dists, 1, flann::SearchParams(32));
	  
	  Vector3d center_tgt(0,0,0);
	  Vector3d center_src = src.GetCenter();

	  int count = 0;
	  for(int i=0; i<srcnum; i++){
//	      if(dists.at<float>(0,i) > max_dist)
//		  continue;
	       structured_indoor_modeling::Point curpt = tgt.GetPoint(searchres.at<int>(0,i));
	       center_tgt += curpt.position;
	       count++;
	  }
	  if(srcnum > 0)
	       center_tgt /= (double)count;
	  else
	       return;

	  // for(int i=0; i<srcnum; i++){
	  //      P(0,i) = src.GetPoint(i).position[0] - center_src[0];
	  //      P(1,i) = src.GetPoint(i).position[1] - center_src[1];
	  //      P(2,i) = src.GetPoint(i).position[2] - center_src[2];

	  //      Q(0,i) = tgt.GetPoint(searchres.at<int>(0,i)).position[0] - center_tgt[0];
	  //      Q(1,i) = tgt.GetPoint(searchres.at<int>(0,i)).position[1] - center_tgt[1];
	  //      Q(2,i) = tgt.GetPoint(searchres.at<int>(0,i)).position[2] - center_tgt[2];
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


