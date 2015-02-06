#include "object_hole_filling.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace structured_indoor_modeling;

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
  int unit = point_cloud.GetNumPoints() / 100;
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

void getSuperpixelLabel(const PointCloud &point_cloud, const Panorama &panorama, const vector<double> &depthmap, const vector<int> &superpixel,const vector< vector<int> >&labelgroup,  vector <int> &superpixelConfidence, int superpixelnum){
    superpixelConfidence.clear();
    superpixelConfidence.resize(superpixelnum);
    for(int i=0;i<superpixelConfidence.size();++i)
	superpixelConfidence[i] = 0;
    int imgwidth = panorama.Width();
    int imgheight = panorama.Height();
    for(int ptid = 0; ptid<point_cloud.GetNumPoints(); ++ptid){
	Vector3d curpt = point_cloud.GetPoint(ptid).position;
	Vector3d localpt = panorama.GlobalToLocal(curpt);
	Vector2d RGBpixel = panorama.Project(curpt);
	Vector2d depth_pixel = panorama.RGBToDepth(RGBpixel);
	double depthv = depthmap[(int)depth_pixel[1] * panorama.DepthWidth() + (int)depth_pixel[0]];
	double curdepth = sqrt(localpt[0]*localpt[0] + localpt[1]*localpt[1] + localpt[2]*localpt[2]);
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
