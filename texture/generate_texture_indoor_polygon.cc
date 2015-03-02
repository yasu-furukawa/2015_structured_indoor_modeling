#include "../base/file_io.h"
#include "generate_texture_indoor_polygon.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

Eigen::Vector3d Patch::UVToManhattan(const Eigen::Vector2d& uv) const {
  return vertices[0] + uv[0] * (vertices[1] - vertices[0]) + uv[1] * (vertices[3] - vertices[0]);
}

Eigen::Vector2d Patch::ManhattanToUV(const Eigen::Vector3d& manhattan) const {
  const double x_length = (vertices[1] - vertices[0]).norm();
  const double y_length = (vertices[3] - vertices[0]).norm();
  
  return Eigen::Vector2d(std::max(0.0, std::min(1.0, (manhattan - vertices[0]).dot(axes[0]) / x_length)),
                         std::max(0.0, std::min(1.0, (manhattan - vertices[0]).dot(axes[1]) / y_length)));
}
  
Eigen::Vector2d Patch::UVToTexture(const Eigen::Vector2d& uv) const {
  return Eigen::Vector2d(texture_size[0] * uv[0], texture_size[1] * uv[1]);
}
  
Eigen::Vector2d Patch::TextureToUV(const Eigen::Vector2d& texture) const {
  return Eigen::Vector2d(texture[0] / texture_size[0], texture[1] / texture_size[1]);
}

double ComputeTexelUnit(const IndoorPolygon& indoor_polygon,
                        const int target_texture_size_for_vertical) {
  return 0.0;
}
  
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
  Vector3d min_xyz, max_xyz;
  for (int v = 0; v < segment.vertices.size(); ++v) {
    for (int i = 0; i < 3; ++i) {
      const double offset = segment.vertices[v].dot(patch->axes[i]);
      if (v == 0) {
        min_xyz[i] = max_xyz[i] = offset;
      } else {
        min_xyz[i] = min(min_xyz[i], offset);
        max_xyz[i] = max(max_xyz[i], offset);
      }
    }
  }
  if (min_xyz[2] != max_xyz[2]) {
    cerr << "Impossible: " << min_xyz[2] << ' ' << max_xyz[2] << endl;
  }
  const double z = min_xyz[2];

  patch->vertices[0] = Vector3d(min_xyz[0], min_xyz[1], z);
  patch->vertices[1] = Vector3d(max_xyz[0], min_xyz[1], z);
  patch->vertices[2] = Vector3d(max_xyz[0], max_xyz[1], z);
  patch->vertices[3] = Vector3d(min_xyz[0], max_xyz[1], z);
  
  // Set texture_size.
  //texture_input....
  
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
