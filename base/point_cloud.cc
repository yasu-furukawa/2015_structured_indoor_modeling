#include <Eigen/Dense>
#include <fstream>
#include "point_cloud.h"

using namespace Eigen;
using namespace std;

namespace base {

PointCloud::PointCloud() {
  
}

bool PointCloud::Init(const file_io::FileIO& file_io, const int panorama) {
  ifstream ifstr;
  ifstr.open(file_io.GetLocalPly(panorama).c_str());

  if (!ifstr.is_open()) {
    ifstr.close();
    return false;
  }

  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 37; ++i)
    ifstr >> stmp;
    
  const int kXOffset = 1;
  const int kYOffset = 1;
  
  depth_width = 0;
  depth_height = 0;

  points.resize(num_points);
  for (auto& point : points) {
    ifstr >> point.depth_position[0] >> point.depth_position[1]
          >> point.position[0] >> point.position[1] >> point.position[2]
          >> point.color[0] >> point.color[1] >> point.color[2]
          >> point.normal[0] >> point.normal[1] >> point.normal[2]
          >> point.intensity;
      
    point.depth_position[0] -= kXOffset;
    point.depth_position[1] -= kYOffset;
      
    depth_width = max(point.depth_position[0] + 1, depth_width);
    depth_height = max(point.depth_position[1] + 1, depth_height);
  }
  ifstr.close();

  return true;
}

void PointCloud::ToGlobal(const file_io::FileIO& file_io, const int panorama) {
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

  for (auto& point : points) {
    Vector4d position4(point.position[0], point.position[1], point.position[2], 1.0);
    position4 = local_to_global * position4;
    point.position = Vector3d(position4[0], position4[1], position4[2]);

    Vector4d normal4(point.normal[0], point.normal[1], point.normal[2], 0.0);
    normal4 = local_to_global * normal4;
    point.normal = Vector3d(normal4[0], normal4[1], normal4[2]);
  }
}

}  // namespace base
