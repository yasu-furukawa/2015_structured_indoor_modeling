#include "../base/file_io.h"
#include "generate_texture_indoor_polygon.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

void SetPatch(const TextureInput& texture_input,
              const Segment& segment,
              const bool visibility_check,
              Patch* patch) {
  // Set patch_axes.
  const Vector3d kUpVector(0, 0, 1);
  
  switch (segment.normal) {
  case Segment::PositiveX: {
    patch->axes[2] = Vector3d(1, 0, 0);
    patch->axes[1] = kUpVector;
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  case Segment::NegativeX: {
    patch->axes[2] = Vector3d(-1, 0, 0);
    patch->axes[1] = kUpVector;
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  case Segment::PositiveY: {
    patch->axes[2] = Vector3d(0, 1, 0);
    patch->axes[1] = kUpVector;
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  case Segment::NegativeY: {
    patch->axes[2] = Vector3d(0, -1, 0);
    patch->axes[1] = kUpVector;
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  case Segment::PositiveZ: {
    patch->axes[2] = Vector3d(0, 0, 1);
    patch->axes[1] = Vector3d(1, 0, 0);
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  case Segment::NegativeZ: {
    patch->axes[2] = Vector3d(0, 0, -1);
    patch->axes[1] = Vector3d(1, 0, 0);
    patch->axes[0] = patch->axes[1].cross(patch->axes[2]);

    break;
  }
  }

  // Set vertices.


  // Set texture_size.
}

void PackTexture(const Patch& patch,
                 Segment* segment,
                 std::vector<std::vector<unsigned char> >* texture_images,
                 std::pair<int, Eigen::Vector2i>* iuv,
                 int* max_texture_height) {



}

void WriteTextureImages(const FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images) {
  for (int t = 0; t < texture_images.size(); ++t) {
    cv::Mat image(texture_image_size, texture_image_size, CV_8UC3);
    int index = 0;
    for (int y = 0; y < texture_image_size; ++y) {
      for (int x = 0; x < texture_image_size; ++x) {
        for (int i = 0; i < 3; ++i) {
          image.at<cv::Vec3b>(y, x)[i] = texture_images[t][index++];
        }
      }
    }

    imwrite(file_io.GetTextureImageIndoorPolygon(t), image);
  }
}

}  // namespace structured_indoor_modeling
