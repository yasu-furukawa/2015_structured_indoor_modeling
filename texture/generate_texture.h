#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/point_cloud.h"

namespace structured_indoor_modeling {

class FileIO;
class Panorama;

// Patch where texture is generated.
struct Patch {
  // ceiling 0------1
  //         |      |
  //         |      |
  // floor   3------2
  Eigen::Vector3d vertices[4];

  // Texture is stored from 0 in a row major format.
  Eigen::Vector2i texture_size;
  std::vector<unsigned char> texture;

  Eigen::Vector2d min_xy_local;
  Eigen::Vector2d max_xy_local;
  
  void InitVertices(const double floor_height) {
    vertices[0] = Eigen::Vector3d(min_xy_local[0], min_xy_local[1], floor_height);
    vertices[1] = Eigen::Vector3d(max_xy_local[0], min_xy_local[1], floor_height);
    vertices[2] = Eigen::Vector3d(max_xy_local[0], max_xy_local[1], floor_height);
    vertices[3] = Eigen::Vector3d(min_xy_local[0], max_xy_local[1], floor_height);
  }

  //----------------------------------------------------------------------  
  Eigen::Vector3d Interpolate(const Eigen::Vector2d& uv) const {
    return vertices[0] + uv[0] * (vertices[1] - vertices[0]) + uv[1] * (vertices[3] - vertices[0]);
  }

  Eigen::Vector2d LocalToTexture(const Eigen::Vector2d& local) const {
    return Eigen::Vector2d(texture_size[0] * (local[0] - min_xy_local[0]) / (max_xy_local[0] - min_xy_local[0]),
                           texture_size[1] * (local[1] - min_xy_local[1]) / (max_xy_local[1] - min_xy_local[1]));
  }
};

int GetEndPanorama(const FileIO& file_io, const int start_panorama);
 
void ReadPanoramas(const FileIO& file_io,
                   const int start_panorama,
                   const int end_panorama,
                   const int num_pyramid_levels,
                   std::vector<std::vector<Panorama> >* panoramas);

void ReadPanoramaToGlobals(const FileIO& file_io,
                           const int start_panorama,
                           const int end_panorama,
                           std::vector<Eigen::Matrix4d>* panorama_to_globals);

void ReadPointClouds(const FileIO& file_io,
                     const int start_panorama,
                     const int end_panorama,
                     std::vector<PointCloud>* point_clouds);
 
void Invert(const std::vector<Eigen::Matrix4d>& panorama_to_globals,
            std::vector<Eigen::Matrix4d>* global_to_panoramas);
 
}  // namespace structured_indoor_modeling
 
