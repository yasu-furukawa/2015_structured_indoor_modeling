#ifndef GENERATE_TEXTURE_H_
#define GENERATE_TEXTURE_H_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace file_io {
  class FileIO;
}  // namespace file_io

class Panorama;

namespace texture {

// Patch where texture is generated.
struct Patch {
  // ceiling 0------1
  //         |      |
  //         |      |
  // floor   3------2
  Eigen::Vector3d vertices[4];

  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;

  // Texture is stored from 0 in a row major format.
  double texel_size;
  int texture_width;
  int texture_height;
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

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                          const Patch& patch,
                          std::vector<std::pair<double, int> >* visible_panoramas_weights);

// double ComputeTexelSize(const std::vector<std::vector<Panorama> >& panoramas);

void SetTextureSize(const int max_texture_size_per_patch,
                    const int texture_height_per_wall,
                    Patch* patch);

void GrabTexture(const std::vector<Panorama>& panorama,
                 Patch* patch);

int ChoosePyramidLevel(const std::vector<Panorama>& panorama,
                       const Patch& patch);

void ConvertPatchToMat(const Patch& patch, cv::Mat* mat);

void PackWallTextures(const std::vector<std::vector<Patch> >& wall_patches,
                      const int texture_image_size,
                      Floorplan* floorplan);

void CopyTextures(const Floorplan& floorplan,
                  const std::vector<std::vector<Patch> >& patches,
                  std::vector<cv::Mat>* texture_images);

void SetFloorPatch(const Floorplan& floorplan,
                   const std::vector<std::vector<Panorama> >& panoramas,
                   const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                   const int max_texture_size_per_floor_patch,
                   std::vector<std::vector<Patch> >* wall_patches);

void SetWallPatches(const Floorplan& floorplan,
                    const std::vector<std::vector<Panorama> >& panoramas,
                    const std::vector<Eigen::Matrix4d>& global_to_panoramas,
                    const int max_texture_size_per_wall_patch,
                    const int texture_height_per_wall,
                    std::vector<std::vector<Patch> >* wall_patches);

void PackFloorTexture(const Patch& floor_patch,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::pair<int, Eigen::Vector2i>* iuv);

void PackWallTextures(const std::vector<std::vector<Patch> >& patches,
                      const int texture_image_size,
                      Floorplan* floorplan,
                      std::pair<int, Eigen::Vector2i>* iuv);

void PackWallTexture(const Patch& patch,
                     const WallTriangulation& wall_triangulation,
                     const int texture_image_size,
                     std::pair<int, Eigen::Vector2i>* iuv);

void UpdateIUV(const Eigen::Vector2i& size,
               const int texture_image_size,
               const std::pair<int, Eigen::Vector2i>& before_iuv,
               std::pair<int, Eigen::Vector2i>* after_iuv);
                
 
}  // namespace texture
 
#endif  // GENERATE_TEXTURE_H_
