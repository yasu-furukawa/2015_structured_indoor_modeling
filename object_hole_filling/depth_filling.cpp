#include "depth_filling.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "../base/file_io.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace structured_indoor_modeling{
  
void DepthFilling::Init(const FileIO &file_io, int id){
  PointCloud point_cloud;
  Panorama panorama;
  point_cloud.Init(file_io,id);
  point_cloud.ToGlobal(file_io, id);
  panorama.Init(file_io,id);
  depthwidth = panorama.DepthWidth();
  depthheight = panorama.DepthHeight();
  depthmap.clear();
  depthmap.resize(depthwidth * depthheight);
  //project the point cloud to the panorama and get the depth

  max_depth =-1e100;
  min_depth = 1e100;
  point_cloud.ToGlobal(file_io, id);
  for(int point=0;point<point_cloud.GetNumPoints();++point){
    Vector3d globalposition = point_cloud.GetPoint(point).position;
    Vector2d pixel = panorama.Project(globalposition);
    Vector2d depth_pixel = panorama.RGBToDepth(pixel);
    Vector3d localposition = panorama.GlobalToLocal(globalposition);
    int depthx = round(depth_pixel[0]+0.5);
    int depthy = round(depth_pixel[1]+0.5);
    if(depthx<0 || depthx>= depthwidth || depthy<0 || depthy>=depthheight)
      continue;
    depthmap[depthy*depthwidth + depthx] = localposition[2];
    if(depthmap[depthy*depthwidth + depthx] < min_depth)
      min_depth = depthmap[depthy*depthwidth+depthx];
    if(depthmap[depthy*depthwidth + depthx] > max_depth)
      max_depth = depthmap[depthy*depthwidth+depthx];
  }
}

void DepthFilling::Init(const PointCloud &point_cloud, const Panorama &panorama){
  depthwidth = panorama.DepthWidth();
  depthheight = panorama.DepthHeight();
  depthmap.clear();
  depthmap.resize(depthwidth * depthheight);
  //project the point cloud to the panorama and get the depth
  max_depth = -1e100;
  min_depth = 1e100;
  for(int point=0;point<point_cloud.GetNumPoints();++point){
    Vector3d globalposition = point_cloud.GetPoint(point).position;
    Vector2d pixel = panorama.Project(globalposition);
    Vector2d depth_pixel = panorama.RGBToDepth(pixel);
    Vector3d localposition = panorama.GlobalToLocal(globalposition);
    int depthx = round(depth_pixel[0]+0.5);
    int depthy = round(depth_pixel[1]+0.5);
    if(depthx<0 || depthx>= depthwidth || depthy<0 || depthy>=depthheight)
      continue;
    depthmap[depthy*depthwidth + depthx] = -1* localposition[0];
    if(depthmap[depthy*depthwidth + depthx] < min_depth)
      min_depth = depthmap[depthy*depthwidth+depthx];
    if(depthmap[depthy*depthwidth + depthx] > max_depth)
      max_depth = depthmap[depthy*depthwidth+depthx];
  }
}

void DepthFilling::fill_hole(){
  
}


void DepthFilling::SaveDepthmap(string path){
  Mat depthimage = Mat(depthheight,depthwidth,CV_8UC3);
  for(int i=0;i<(int)depthmap.size();++i){
    int curv = (int)((depthmap[i] - min_depth) / (max_depth - min_depth) * 255);
    Vec3b curpix = Vec3b((uchar)curv, (uchar)curv, (uchar)curv);
    depthimage.at<Vec3b>(i / depthwidth, i % depthwidth) = curpix;
  }
  imwrite(path, depthimage);
  waitKey(10);
}

 
}//namespace
