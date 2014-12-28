#ifndef GENERATE_TEXTURE_H_
#define GENERATE_TEXTURE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace file_io {
  class FileIO;
}  // namespace file_io

class Floorplan;
class Panorama;
class WallTriangulation;

namespace texture {

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

  //----------------------------------------------------------------------  
  Eigen::Vector3d Interpolate(const Eigen::Vector2d& uv) const {
    return vertices[0] + uv[0] * (vertices[1] - vertices[0]) + uv[1] * (vertices[3] - vertices[0]);
  }
};

void ReadPanoramas(const file_io::FileIO& file_io,
                   const int num_panoramas,
                   const int num_pyramid_levels,
                   std::vector<std::vector<Panorama> >* panoramas);

void ReadPanoramaToGlobals(const file_io::FileIO& file_io,
                           const int num_panoramas,
                           std::vector<Eigen::Matrix4d>* panorama_to_globals);

void Invert(const std::vector<Eigen::Matrix4d>& panorama_to_globals,
            std::vector<Eigen::Matrix4d>* global_to_panoramas);


void PackWallTextures(const std::vector<std::vector<Patch> >& wall_patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      int* max_texture_height);

// Walls.
void SetWallPatches(const Floorplan& floorplan,
                    const std::vector<std::vector<Panorama> >& panoramas,
                    const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                    const int max_texture_size_per_wall_patch,
                    const int texture_height_per_wall,
                    std::vector<std::vector<Patch> >* wall_patches);

void PackWallTextures(const std::vector<std::vector<Patch> >& patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height);
 
// Floor.
void SetFloorPatch(const Floorplan& floorplan,
                   const std::vector<std::vector<Panorama> >& panoramas,
                   const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                   const int max_texture_size_per_floor_patch,
                   Patch* floor_patch,
                   Eigen::Vector2d* min_xy_local,
                   Eigen::Vector2d* max_xy_local);

void PackFloorTexture(const Patch& floor_patch,
                      const Eigen::Vector2d& min_xy_local,
                      const Eigen::Vector2d& max_xy_local,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::vector<std::vector<unsigned char> >* texture_images,
                      std::pair<int, Eigen::Vector2i>* iuv,
                      int* max_texture_height);

void WriteTextureImages(const file_io::FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images);
 
}  // namespace texture
 
#endif  // GENERATE_TEXTURE_H_
