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


inline float gaussian(double x, double sigma){
  //  return 1.0/(sigma*std::sqrt(2*PI)) * std::exp(-1*(x*x/(2*sigma*sigma)));
  return std::exp(-1*(x*x/(2*sigma*sigma)));
}


double diffFunc(int pix1,int pix2, const vector<int>&superpixelConfidence){
  return gaussian(1.0 / (abs((double)superpixelConfidence[pix1] - (double)superpixelConfidence[pix2]) + 0.001), 1);
}

MRF::CostVal funcCost(int pix1,int pix2,int i,int j){
  if(i == j)
    return 0;
  else
    return 1;
}

double colorDiffFunc(int pix1,int pix2, const vector <Vector3d>&averageRGB){
    Vector3d colordiff = averageRGB[pix1] - averageRGB[pix2];
    return colordiff.norm();
}

void MRFOptimizeLabels(const vector<int>&superpixelConfidence,  const map<pair<int,int>,int> &pairmap, const vector<Vector3d>&averageRGB, float smoothnessweight, vector <int> &superpixelLabel){
  int superpixelnum = superpixelConfidence.size();
  vector<MRF::CostVal>data(superpixelnum * 2);
  vector<MRF::CostVal>smooth(4);
  //model

  for(int i=0;i<superpixelnum;i++){
    data[2*i] = (MRF::CostVal)(gaussian(1.0/((float)superpixelConfidence[i] + 0.001), 1) * 1000) ;    //assign 0
    data[2*i+1] = (MRF::CostVal)(gaussian((float)superpixelConfidence[i], 1) * 1000);  //assign 1
  }
  smooth[0] = 1; smooth[3] = 1;
  smooth[1] = 0; smooth[2] = 0;

  DataCost *dataterm = new DataCost(&data[0]);
  SmoothnessCost *smoothnessterm = new SmoothnessCost(&smooth[0]);
  EnergyFunction *energy = new EnergyFunction(dataterm,smoothnessterm);

  MRF *mrf;
  mrf = new Expansion(superpixelnum, 2, energy);

  //solve
  mrf->initialize();

  for(auto mapiter:pairmap){
    pair<int,int> curpair = mapiter.first;
//    MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)(diffFunc(curpair.first,curpair.second,superpixelConfidence) * 1000);
    MRF::CostVal weight = (MRF::CostVal)mapiter.second * (MRF::CostVal)(colorDiffFunc(curpair.first,curpair.second,averageRGB) * 0.05);
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
