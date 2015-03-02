#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../base/indoor_polygon.h"
#include "../base/point_cloud.h"
#include "generate_texture.h"

namespace structured_indoor_modeling {

// All the coordinates are in the floorplan coordinate frame.
struct Patch {
  Segment segment;

  //      patch xaxis
  //       0------1
  // patch |      |
  // yaxis |      |
  //       3------2
  //
  // patch zaxis is the normal.
  
  // Axis aligned bounding box.
  Eigen::Vector3d patch_axes[3];

  Eigen::Vector3d vertices[4];
  Eigen::Vector2i texture_size;
  std::vector<unsigned char> texture;

  Eigen::Vector3d Interpolate(const Eigen::Vector2d& uv) const {
    return vertices[0] + uv[0] * (vertices[1] - vertices[0]) + uv[1] * (vertices[3] - vertices[0]);
  }

  Eigen::Vector2d LocalToTexture(const Eigen::Vector2d& local) const {
    return Eigen::Vector2d(texture_size[0] * (local[0] - min_xy_local[0]) / (max_xy_local[0] - min_xy_local[0]),
                           texture_size[1] * (local[1] - min_xy_local[1]) / (max_xy_local[1] - min_xy_local[1]));
  }
  




};
  
// Input data from cli.cc.
struct TextureInput {
  IndoorPolygon indoor_polygon;
  std::vector<std::vector<Panorama> > panoramas;
  std::vector<PointCloud> point_clouds;
  int pyramid_level;
  int max_texture_size_per_floor_patch;
  int max_texture_size_per_non_floor_patch;
  double position_error_for_floor;
  double patch_size_for_synthesis;
  int num_cg_iterations;
};

 


}  // namespace structured_indoor_modeling
