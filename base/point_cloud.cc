#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "file_io.h"
#include "point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

PointCloud::PointCloud() {
}

bool PointCloud::Init(const FileIO& file_io, const int panorama) {
  return Init(file_io.GetLocalPly(panorama).c_str());
}

bool PointCloud::Init(const std::string& filename) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    ifstr.close();
    return false;
  }

  bool has_object_id = false;
  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_points, itemp, object_num = -1;
  ifstr >> num_points;
  for (int i = 0; i < 36; ++i)
    ifstr >> stmp;

  ifstr >> stmp;
  if(stmp == "property") {
    has_object_id = true;
    for(int i = 0; i < 3; ++i)
      ifstr >> stmp;
  }
    
  const int kXOffset = 1;
  const int kYOffset = 1;
  
  depth_width = 0;
  depth_height = 0;

  points.resize(num_points);

  //to handle different point format
  for (auto& point : points) {
    ifstr >> point.depth_position[0] >> point.depth_position[1]
          >> point.position[0] >> point.position[1] >> point.position[2]
          >> point.color[0] >> point.color[1] >> point.color[2]
          >> point.normal[0] >> point.normal[1] >> point.normal[2]
          >> point.intensity;
    
    if (has_object_id) {
      ifstr >> point.object_id;
      object_num = point.object_id > object_num ? point.object_id : object_num;
    } else {
      point.object_id = -1;
    }
    
    point.depth_position[0] -= kXOffset;
    point.depth_position[1] -= kYOffset;
      
    depth_width = max(point.depth_position[0] + 1, depth_width);
    depth_height = max(point.depth_position[1] + 1, depth_height);
  }
  
  ifstr.close();

  if(has_object_id)
    num_object = object_num + 1;

  return true;
}

void PointCloud::Write(const std::string& filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  if (!ofstr.is_open()) {
    cerr << "Failed in writing: " << filename << endl;
    exit (1);
  }

  const int kXOffset = 1;
  const int kYOffset = 1;
  
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
	<< "property uchar object_id"<<endl
        << "end_header" << endl;
  for (const auto& point : points) {
    ofstr << point.depth_position[0] + kXOffset << ' '
          << point.depth_position[1] + kYOffset << ' '
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
