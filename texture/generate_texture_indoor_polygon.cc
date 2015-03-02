#include "generate_texture_indoor_polygon.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

void SetPatch(const TextureInput& texture_input,
              const Segment& segment,
              Patch* patch) {



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



}

}  // namespace structured_indoor_modeling
