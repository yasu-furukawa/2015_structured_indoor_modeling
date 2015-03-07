#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../base/indoor_polygon.h"
#include "../base/point_cloud.h"
#include "generate_texture.h"

namespace structured_indoor_modeling {

// All the coordinates are in the manhattan coordinate frame.
struct Patch {
  //      patch xaxis
  //       0------1
  // patch |      |
  // yaxis |      |
  //       3------2
  //
  // patch zaxis is the normal.
  
  // Axis aligned bounding box.
  Eigen::Vector3d axes[3];

  Eigen::Vector3d vertices[4];
  Eigen::Vector2i texture_size;
  std::vector<unsigned char> texture;

  Eigen::Vector3d UVToManhattan(const Eigen::Vector2d& uv) const;
  Eigen::Vector2d ManhattanToUV(const Eigen::Vector3d& manhattan) const;
  Eigen::Vector2d UVToTexture(const Eigen::Vector2d& uv) const;
  Eigen::Vector2d TextureToUV(const Eigen::Vector2d& texture) const;
};
  
// Input data from cli.cc.
struct TextureInput {
  IndoorPolygon indoor_polygon;
  std::vector<std::vector<Panorama> > panoramas;
  std::vector<PointCloud> point_clouds;

  std::vector<std::vector<double> > panorama_depths;

  int pyramid_level;
  int max_texture_size_per_floor_patch;
  int max_texture_size_per_non_floor_patch;
  double position_error_for_floor;
  double patch_size_for_synthesis;
  int num_cg_iterations;

  double texel_unit;
};

double ComputeTexelUnit(const IndoorPolygon& indoor_polygon,
                        const int target_texture_size_for_vertical);

void ComputePanoramaDepths(TextureInput* texture_input);

 void SmoothField(const int width, const int height, const double hole,
                  std::vector<double>* field);
 
void SetPatch(const TextureInput& texture_input,
              const Segment& segment,
              const bool visibility_check,
              Patch* patch);

void PackTexture(const Patch& patch,
                 const int texture_image_size,
                 Segment* segment,
                 std::vector<std::vector<unsigned char> >* texture_images,
                 std::pair<int, Eigen::Vector2i>* iuv,
                 int* max_texture_height);

void WriteTextureImages(const FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images);

}  // namespace structured_indoor_modeling
