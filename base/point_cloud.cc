#include <Eigen/Dense>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <limits>
#include "file_io.h"
#include "point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int PointCloud::kDepthPositionOffset = 1;

PointCloud::PointCloud() {
  InitializeMembers();
}

void PointCloud::InitializeMembers() {
  center.resize(3);
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;

  bounding_box.resize(6);
  bounding_box[0] = numeric_limits<double>::max();
  bounding_box[2] = numeric_limits<double>::max();
  bounding_box[4] = numeric_limits<double>::max();
  bounding_box[1] = -numeric_limits<double>::max();
  bounding_box[3] = -numeric_limits<double>::max();
  bounding_box[5] = -numeric_limits<double>::max();

  depth_width = 0;
  depth_height = 0;
  num_objects = 0;
}

bool PointCloud::Init(const FileIO& file_io, const int panorama) {
  return Init(file_io.GetLocalPly(panorama).c_str());
}

bool PointCloud::Init(const std::string& filename) {
  InitializeMembers();
  
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    ifstr.close();
    return false;
  }

  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 36; ++i)
    ifstr >> stmp;

  ifstr >> stmp;
  bool has_object_id = false;
  if(stmp == "property") {
    has_object_id = true;
    for(int i = 0; i < 3; ++i)
      ifstr >> stmp;
  }
    
  points.clear();
  points.resize(num_points);

  const int kInvalidObjectId = -1;

  // To handle different point format.
  for (auto& point : points) {
    ifstr >> point.depth_position[0] >> point.depth_position[1]
          >> point.position[0] >> point.position[1] >> point.position[2]
          >> point.color[0] >> point.color[1] >> point.color[2]
          >> point.normal[0] >> point.normal[1] >> point.normal[2]
          >> point.intensity;

    if (has_object_id){
      ifstr >> point.object_id;
    } else {
      point.object_id = kInvalidObjectId;
    }
    
    point.depth_position[0] -= kDepthPositionOffset;
    point.depth_position[1] -= kDepthPositionOffset;
  }

  ifstr.close();

  Update();

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
        << "element vertex " << (int)points.size() << endl
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

  for (const auto& point : points) {
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

void PointCloud::WriteObject(const string& filename, const int objectid){
    vector<Point>object_points;
    for(const auto&pt: points){
	if(pt.object_id == objectid)
	    object_points.push_back(pt);
    }
    
    PointCloud objectcloud;
    objectcloud.AddPoints(object_points);
    objectcloud.Write(filename);
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
     AddPoints(point_cloud.points);
}


void PointCloud::AddPoints(const vector<Point>& new_points) {
     int orinum = points.size();
  points.insert(points.end(), new_points.begin(), new_points.end());
  for(int i=orinum;i<points.size();i++){
       points[i].object_id += num_objects;
  }
  Update();
}

void PointCloud::RemovePoints(const std::vector<int>& indexes) {
  vector<bool> keep(points.size(), true);
  for (const auto& index : indexes) {
    assert(0 <= index && index < (int)points.size());
    keep[index] = false;
  }
  
  vector<Point> new_points;
  for (int p = 0; p < (int)points.size(); ++p) {
    if (keep[p])
      new_points.push_back(points[p]);
  }

  new_points.swap(points);
  Update();
}

  // yasu Should the input type be float instead of int?
void PointCloud::SetAllColor(float r,float g,float b){
  for(auto &v: points){
    v.color[0] = r;
    v.color[1] = g;
    v.color[2] = b;
  }
}

// yasu Should the input type (rgb) be float instead of int?
void PointCloud::SetColor(int ind, float r,float g,float b){
     assert(ind >= 0 && ind < (int)points.size());
     points[ind].color[0] = r;
     points[ind].color[1] = g;
     points[ind].color[2] = b;
}

// yasu To follow the convention in the class, this name should be
// really Update instead of update. This update makes it very
// difficult to understand this class.  It is not very clear at first
// sight, what members are reset, and what members are not. Have to
// read the comment carefully and compare against the header file.
// Ideally, one should be able to understand the functionality of a
// function easily without reading the code or a comment carefully.
//
// To be simple, this class should update all the variables in my opinion.
//
// 
// Update point cloud center, depth_width, depth_height, bounding_box. 
// Note that num_object is not changed, to avoid confusing.
void PointCloud::Update(){
  InitializeMembers();
  
  // To handle different point format.
  for (const auto& point : points) {
    center += point.position;

    depth_width = max(point.depth_position[0] + 1, depth_width);
    depth_height = max(point.depth_position[1] + 1, depth_height);

    num_objects = max(num_objects, point.object_id + 1);
    
    bounding_box[0] = min(point.position[0],bounding_box[0]);
    bounding_box[1] = max(point.position[0],bounding_box[1]);
    bounding_box[2] = min(point.position[1],bounding_box[2]);
    bounding_box[3] = max(point.position[1],bounding_box[3]);
    bounding_box[4] = min(point.position[2],bounding_box[4]);
    bounding_box[5] = max(point.position[2],bounding_box[5]);
  }
  if (!points.empty())
    center /= (int)points.size();
}
   
double PointCloud::GetBoundingboxVolume(){
  if(bounding_box.size() == 0)
    return 0;
  return (bounding_box[1]-bounding_box[0])*(bounding_box[3]-bounding_box[2])*(bounding_box[5]-bounding_box[4]);
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
  Update();
}

//----------------------------------------------------------------------
void ReadPointClouds(const FileIO& file_io, std::vector<PointCloud>* point_clouds) {
  cout << "Reading pointclouds" << flush;
  const int num_panoramas = GetNumPanoramas(file_io);

  point_clouds->clear();
  point_clouds->resize(num_panoramas);
  for (int p = 0; p < num_panoramas; ++p) {
    cout << '.' << flush;
    point_clouds->at(p).Init(file_io, p);
    point_clouds->at(p).ToGlobal(file_io, p);
  }
  cout << " done." << endl;
}  
  
}  // namespace structured_indoor_modeling
