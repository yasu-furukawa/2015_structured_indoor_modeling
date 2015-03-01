#ifndef GENERATE_TEXTURE_H_
#define GENERATE_TEXTURE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../base/floorplan.h"
#include "../base/point_cloud.h"
#include "generate_texture.h"

namespace structured_indoor_modeling {

class FileIO;
class Panorama;
class WallTriangulation;

// Input data from cli.cc.
struct TextureInput {
  Floorplan floorplan;
  std::vector<std::vector<Panorama> > panoramas;
  std::vector<PointCloud> point_clouds;
  int pyramid_level_for_floor;
  int max_texture_size_per_floor_patch;
  int max_texture_size_per_wall_patch;
  int texture_height_per_wall;
  double position_error_for_floor;
  double patch_size_for_synthesis;
  int num_cg_iterations;
};

void PackWallTextures(const std::vector<std::vector<Patch> >& wall_patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      int* max_texture_height);

// Walls.
void SetWallPatches(const TextureInput& texture_input,
                    std::vector<std::vector<Patch> >* wall_patches);

void PackWallTextures(const std::vector<std::vector<Patch> >& patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height);
 
// Floor.
void SetFloorPatch(const TextureInput& texture_input, Patch* floor_patch);

void PackFloorTexture(const Patch& floor_patch,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height);

void WriteTextureImages(const FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images);
 
}  // namespace structured_indoor_modeling
 
#endif  // GENERATE_TEXTURE_H_
