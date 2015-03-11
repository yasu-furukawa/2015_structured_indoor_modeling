#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <assert.h>
#include "file_io.h"
#include "point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int PointCloud::kDepthPositionOffset = 1;

PointCloud::PointCloud() {
    boundingbox.resize(6);
    center.resize(3);
    boundingbox[0] = 1e100; boundingbox[2] = 1e100; boundingbox[4] = 1e100;
    boundingbox[1] = -1e100; boundingbox[3] = -1e100; boundingbox[5] = -1e100;
    depth_width = 0;
    depth_height = 0;
    num_objects = 0;
    valid_points_num = 0;
    has_object_id = true;
    center[0] = 0; center[1] = 0; center[2] = 0;
}

bool PointCloud::Init(const FileIO& file_io, const int panorama) {
  return Init(file_io.GetLocalPly(panorama).c_str());
}

bool PointCloud::Init(const std::string& filename) {
  num_objects = 1;
  
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    ifstr.close();
    return false;
  }

  center[0] = 0.0;center[1] = 0.0;center[2] = 0.0;
  
  bool has_object_id = false;

  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 36; ++i)
    ifstr >> stmp;

  ifstr >> stmp;
  if(stmp == "property") {
    has_object_id = true;
    for(int i = 0; i < 3; ++i)
      ifstr >> stmp;
  }
    
  depth_width = 0;
  depth_height = 0;
  boundingbox[0] = 1e100;  boundingbox[1] = -1e100; boundingbox[2] = 1e100; boundingbox[3] = -1e100;
  boundingbox[4] = 1e100; boundingbox[5] = -1e100;
  
  points.clear();
  points.resize(num_points);
  
  mask.resize(num_points);
  for(auto &maskv: mask)
       maskv = 1;
  valid_points_num = num_points;
  
  const int kInvalidObjectId = -1;

  // To handle different point format.
  for (auto& point : points) {
    ifstr >> point.depth_position[0] >> point.depth_position[1]
          >> point.position[0] >> point.position[1] >> point.position[2]
          >> point.color[0] >> point.color[1] >> point.color[2]
          >> point.normal[0] >> point.normal[1] >> point.normal[2]
          >> point.intensity;

    center += point.position;
    if(has_object_id){
      ifstr >> point.object_id;
    } else {
      point.object_id = kInvalidObjectId;
    }
    
    point.depth_position[0] -= kDepthPositionOffset;
    point.depth_position[1] -= kDepthPositionOffset;

    boundingbox[0] = min(point.position[0],boundingbox[0]);
    boundingbox[1] = max(point.position[0],boundingbox[0]);
    boundingbox[2] = min(point.position[1],boundingbox[1]);
    boundingbox[3] = max(point.position[1],boundingbox[1]);
    boundingbox[4] = min(point.position[2],boundingbox[2]);
    boundingbox[5] = max(point.position[2],boundingbox[2]);
    
    depth_width = max(point.depth_position[0] + 1, depth_width);
    depth_height = max(point.depth_position[1] + 1, depth_height);
  }

  center /= (double)num_points;
  
  ifstr.close();

  // Setting num_objects.
  if(has_object_id) {
    for (const auto& point : points) {
      num_objects = max(num_objects, point.object_id);
    }
    ++num_objects;
  }

  return true;
}
  
void PointCloud::Write(const std::string& filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  if (!ofstr.is_open()) {
    cerr << "Failed in writing: " << filename << endl;
    exit (1);
  }

  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "element vertex " << valid_points_num << endl
        << "property int height" << endl
        << "property int width" << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property uchar red" << endl
        << "property uchar green" << endl
        << "property uchar blue" << endl
        << "property float nx" << endl
        << "property float ny" << endl
        << "property float nz" << endl
        << "property uchar intensity" << endl
	<< "property uchar object_id" << endl
	<< "end_header" << endl;
  
  for (int i=0; i<points.size(); i++) {
       if(mask[i] == 0)
	    continue;
       structured_indoor_modeling::Point point = points[i];
    ofstr << point.depth_position[0] + kDepthPositionOffset << ' '
          << point.depth_position[1] + kDepthPositionOffset << ' '
          << point.position[0] << ' '
          << point.position[1] << ' '
          << point.position[2] << ' '
          << (int)point.color[0] << ' '
          << (int)point.color[1] << ' '
          << (int)point.color[2] << ' '
          << point.normal[0] << ' '
          << point.normal[1] << ' '
          << point.normal[2] << ' '
          << point.intensity << ' '
	  << point.object_id << endl;
  }

  ofstr.close();
}

void PointCloud::Rotate(const Eigen::Matrix3d& rotation) {
  Matrix4d transformation;
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      transformation(y, x) = rotation(y, x);
    }
    transformation(y, 3) = 0.0;
  }
  transformation(3, 0) = 0;
  transformation(3, 1) = 0;
  transformation(3, 2) = 0;
  transformation(3, 3) = 1;

  Transform(transformation);
}
  
void PointCloud::ToGlobal(const FileIO& file_io, const int panorama) {
  Matrix4d local_to_global;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetLocalToGlobalTransformation(panorama).c_str());
    
    char ctmp;
    ifstr >> ctmp;
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 4; ++x) {
        ifstr >> local_to_global(y, x);
      }
    }
    ifstr.close();
    local_to_global(3, 0) = 0;
    local_to_global(3, 1) = 0;
    local_to_global(3, 2) = 0;
    local_to_global(3, 3) = 1;
  }
  Transform(local_to_global);
}

void PointCloud::AddPoints(const PointCloud& point_cloud){
     for(int i=0;i<point_cloud.GetNumPoints();i++)
	  mask.push_back(point_cloud.GetMask(i));

    points.insert(points.end(), point_cloud.points.begin(), point_cloud.points.end());
    num_objects += point_cloud.GetNumObjects();
    valid_points_num += point_cloud.GetValidPointsNum();
    update();
}

//add an array of Point to the point cloud. Update each member variable, a little faster than calling update()......
void PointCloud::AddPoints(const vector<Point>& new_points){
    int orilength = points.size();
    
    for(int i=0;i<new_points.size();i++)
	 mask.push_back(1);
    valid_points_num += new_points.size();
    
    center = center * (double)orilength;
    int oriobjectnum = num_objects;
    points.insert(points.end(), new_points.begin(), new_points.end());

    for(int i=orilength;i<(int)points.size();i++){
	points[i].object_id += oriobjectnum;
	center += points[i].position;
	num_objects = std::max(num_objects, points[i].object_id);
	depth_width = std::max(depth_width,points[i].depth_position[0]);
	depth_height = std::max(depth_height,points[i].depth_position[1]);
	boundingbox[0] = std::min(boundingbox[0],points[i].position[0]);
	boundingbox[1] = std::max(boundingbox[1],points[i].position[0]);
	boundingbox[2] = std::min(boundingbox[2],points[i].position[1]);
	boundingbox[3] = std::max(boundingbox[3],points[i].position[1]);
	boundingbox[4] = std::min(boundingbox[4],points[i].position[2]);
	boundingbox[5] = std::max(boundingbox[5],points[i].position[2]);
    }
    center = center / (double)points.size();
}

void PointCloud::RemovePoint(int ind, bool isupdate){
    assert(ind>=0 && ind < points.size());
    if(mask[ind] == 0)
	 return;
    mask[ind] = 0;
    valid_points_num --;
    if(isupdate)
	update();
}

void PointCloud::SetAllColor(int r,int g,int b){
     for(auto &v: points){
	  v.color[0] = (float)r;
	  v.color[1] = (float)g;
	  v.color[2] = (float)b;
     }
}

void PointCloud::SetColor(int ind, int r,int g,int b){
     assert(ind < points.size());
     points[ind].color[0] = float(r);
     points[ind].color[1] = float(g);
     points[ind].color[2] = float(b);
}

//update point cloud center, depth_width, depth_height, boundingbox. Note that num_object is not changed, to avoid confusing
void PointCloud::update(){
    depth_width = 0;
    depth_height = 0;
    boundingbox[0] = 1e100;  boundingbox[1] = -1e100; boundingbox[2] = 1e100; boundingbox[3] = -1e100;
    boundingbox[4] = 1e100; boundingbox[5] = -1e100;
  
    // To handle different point format.
    for (int i=0; i<points.size(); i++) {
	 if(mask[i] == 0)
	      continue;
	 structured_indoor_modeling::Point point = points[i];
	 center += point.position;
	 boundingbox[0] = min(point.position[0],boundingbox[0]);
	 boundingbox[1] = max(point.position[0],boundingbox[0]);
	 boundingbox[2] = min(point.position[1],boundingbox[1]);
	 boundingbox[3] = max(point.position[1],boundingbox[1]);
	 boundingbox[4] = min(point.position[2],boundingbox[2]);
	 boundingbox[5] = max(point.position[2],boundingbox[2]);
	 
	 depth_width = max(point.depth_position[0] + 1, depth_width);
	 depth_height = max(point.depth_position[1] + 1, depth_height);
    }
    center /= (double)valid_points_num;
}
    
double PointCloud::GetBoundingboxVolume(){
  if(boundingbox.size() == 0)
    return 0;
  return (boundingbox[1]-boundingbox[0])*(boundingbox[3]-boundingbox[2])*(boundingbox[5]-boundingbox[4]);
}
  
void PointCloud::Transform(const Eigen::Matrix4d& transformation) {
  for (auto& point : points) {
    Vector4d position4(point.position[0], point.position[1], point.position[2], 1.0);
    position4 = transformation * position4;
    point.position = Vector3d(position4[0], position4[1], position4[2]);

    Vector4d normal4(point.normal[0], point.normal[1], point.normal[2], 0.0);
    normal4 = transformation * normal4;
    point.normal = Vector3d(normal4[0], normal4[1], normal4[2]);
  }
}

}  // namespace structured_indoor_modeling
