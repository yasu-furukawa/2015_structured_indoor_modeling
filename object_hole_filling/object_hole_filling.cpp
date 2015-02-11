#include "object_hole_filling.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

#define PI 3.1415927

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


void labelTolabelgroup(const std::vector<int>& labels, std::vector< std::vector<int> >&labelgroup, int numgroup){
  labelgroup.resize(numgroup);
  for(int i=0;i<labels.size();i++){
    labelgroup[labels[i]].push_back(i);
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
  int objectnum = 0;
  vector <Vector3f> colors;
  int totalnum = point_cloud.GetNumPoints();
  for(int i=0;i<point_cloud.GetNumPoints();++i){
    structured_indoor_modeling::Point curpt = point_cloud.GetPoint(i);
    Vector3f curcolor = curpt.color;
    bool flag = true;
    for(int j=0;j<colors.size();++j){
      if(curcolor == colors[j]){
	objectgroup[j].push_back(i);
	flag = false;
      }
    }
    if(flag){
      objectgroup.resize(objectgroup.size() + 1);
      objectgroup.back().push_back(i);
      colors.push_back(curcolor);
      objectnum++;
    }
  }
  return objectnum;
}

void getSuperpixelLabel(const PointCloud &point_cloud,const vector<int> &objectgroup,  const Panorama &panorama, const vector<double> &depthmap, const vector<int> &superpixel,const vector< vector<int> >&labelgroup,  vector <int> &superpixelConfidence, int superpixelnum){
    superpixelConfidence.clear();
    superpixelConfidence.resize(superpixelnum);
    for(int i=0;i<superpixelConfidence.size();++i)
	superpixelConfidence[i] = 0;
    int imgwidth = panorama.Width();
    int imgheight = panorama.Height();

    for(int ptid = 0; ptid<objectgroup.size(); ++ptid){
      	Vector3d curpt = point_cloud.GetPoint(objectgroup[ptid]).position;
	Vector3d panCenter = panorama.GetCenter();
	Vector3d offset = curpt - panCenter;
	Vector2d RGBpixel = panorama.Project(curpt);
	Vector2d depth_pixel = panorama.RGBToDepth(RGBpixel);
	double depthv = depthmap[(int)depth_pixel[1] * panorama.DepthWidth() + (int)depth_pixel[0]];
	double curdepth = offset.norm();
	//visibility test
	if(curdepth > depthv)
	  continue;
	int superpixellabel = superpixel[(int)RGBpixel[1] * imgwidth + (int)RGBpixel[0]];
	superpixelConfidence[superpixellabel] += 1;
    }
    // for(int i=0;i<superpixelConfidence.size();++i){
    // 	if(superpixelConfidence[i] < (int) labelgroup[i].size() * 0.4)
    // 	    superpixelConfidence[i] = 0;
    // }
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
      auto iter = pairmap.find(pair<int,int>(minlabelx,maxlabelx));
      if(iter != pairmap.end())
	iter->second += 1;
      else
	pairmap.insert(pair<pair<int,int>,int>(pair<int,int>(minlabelx,maxlabelx),1));

      iter = pairmap.find(pair<int,int>(minlabely,maxlabely));
      if(iter != pairmap.end())
	iter->second += 1;
      else
	pairmap.insert(pair<pair<int,int>,int>(pair<int,int>(minlabely,maxlabely),1));
    }
  }
}


inline double gaussian(double x, double sigma){
  return 1.0/(sigma*std::sqrt(2*PI)) * std::exp(-1*(x*x/(2*sigma*sigma)));
}


double diffFunc(int pix1,int pix2, const vector<int>&superpixelConfidence){
  return gaussian(1.0 / (abs((double)superpixelConfidence[pix1] - (double)superpixelConfidence[pix2]) + 0.001), 1);
}

MRF::CostVal funcCost(int pix1,int pix2,int i,int j){
  if(i == j)
    return 0.0;
  else
    return 1.0;
}

void MRFOptimizeLabels(const vector<int>&superpixelConfidence,  const map<pair<int,int>,int> &pairmap, float smoothnessweight, vector <int> &superpixelLabel){
  int superpixelnum = superpixelConfidence.size();
  vector<MRF::CostVal>data(superpixelnum * 2);
  vector<MRF::CostVal>smooth(4);
  //model
  for(int i=0;i<superpixelnum;i++){
    data[2*i] = (MRF::CostVal)gaussian(1.0/((double)superpixelConfidence[i] + 0.001), 1) ;    //assign 0
    data[2*i+1] = (MRF::CostVal)gaussian((double)superpixelConfidence[i], 1);  //assign 1
  }
  
  smooth[0] = 0; smooth[3] = 0;
  smooth[1] = 1; smooth[2] = 1;
   
  DataCost *dataterm = new DataCost(&data[0]);
  SmoothnessCost *smoothnessterm = new SmoothnessCost(funcCost);
  EnergyFunction *energy = new EnergyFunction(dataterm,smoothnessterm);

  MRF *mrf;
  mrf = new Expansion(superpixelnum, 2, energy);

  //solve
  mrf->initialize();
    
  for(auto mapiter:pairmap){
    pair<int,int> curpair = mapiter.first;
    MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)diffFunc(curpair.first,curpair.second,superpixelConfidence);
    mrf->setNeighbors(curpair.first,curpair.second, weight);
  }

  mrf->clearAnswer();
  
  for(int i=0;i<superpixelnum;i++)
    mrf->setLabel(i,0);

  MRF::EnergyVal E;
  E = mrf->totalEnergy();

  printf("Energy at the Start= %g (%g,%g)\n",(float)E,(float)mrf->smoothnessEnergy(),(float)mrf->dataEnergy());

  float t,tot_t = 0;
  for(int iter=0;iter<6;iter++){
    mrf->optimize(10,t);
    tot_t = tot_t + t;
    printf("energy = %g (%f secs)\n",(float)E,tot_t);
  }
  
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
