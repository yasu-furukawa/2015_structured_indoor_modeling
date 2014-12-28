#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <gflags/gflags.h>


#include "../calibration/file_io.h"
#include "../floorplan/floorplan.h"
#include "../floorplan/panorama.h"
#include "generate_texture.h"

DEFINE_int32(num_panoramas, 1, "Number of panorama images.");
DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
// DEFINE_double(texel_size_rescale, 1.0, "Less than 1 to increase resolution.");
DEFINE_int32(max_texture_size_per_floor_patch, 2048, "Maximum texture size for each floor patch.");
DEFINE_int32(max_texture_size_per_wall_patch, 1024, "Maximum texture size for each wall patch.");
DEFINE_int32(texture_height_per_wall, 512, "Texture height for each wall patch.");
DEFINE_int32(texture_image_size, 2048, "Texture image size to be written.");

using namespace Eigen;
using namespace std;
using namespace texture;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InitGoogleLogging(argv[0]);

  // Read data from the directory.
  file_io::FileIO file_io(argv[1]);

  vector<vector<Panorama> > panoramas;
  {
    ReadPanoramas(file_io,
                  FLAGS_num_panoramas,
                  FLAGS_num_pyramid_levels,
                  &panoramas);
  }

  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramaToGlobals(file_io, FLAGS_num_panoramas, &panorama_to_globals);
  }

  Floorplan floorplan;
  {
    const string filename = file_io.GetFloorplan();
    ifstream ifstr;
    ifstr.open(filename.c_str());
    ifstr >> floorplan;
    ifstr.close();
  }

  vector<Matrix4d> global_to_panoramas;
  {
    Invert(panorama_to_globals, &global_to_panoramas);
  }

  // Unit for a texel.
  // const double texel_size = ComputeTexelSize(panoramas) * FLAGS_texel_size_rescale;
  
  // For each wall rectangle, floor, and ceiling,
  // 0. Identify visible panoramas.
  // 1. Grab texture
  // 2. Stitch
  // Floor texture.
  Patch floor_patch;
  Vector2d min_xy_local, max_xy_local;
  SetFloorPatch(floorplan,
                panoramas,
                global_to_panoramas,
                FLAGS_max_texture_size_per_floor_patch,
                &floor_patch,
                &min_xy_local,
                &max_xy_local);

  // Wall textures.
  vector<vector<Patch> > wall_patches;
  SetWallPatches(floorplan,
                 panoramas,
                 global_to_panoramas,
                 FLAGS_max_texture_size_per_wall_patch,
                 FLAGS_texture_height_per_wall,
                 &wall_patches);

  // Texture image.
  vector<vector<unsigned char> > texture_images;
  // Texture coordinate.
  pair<int, Vector2i> iuv(0, Vector2i(0, 0));
  int max_texture_height = 0;

  // Set texture coordinates.
  PackFloorTexture(floor_patch, min_xy_local, max_xy_local, FLAGS_texture_image_size,
                   &floorplan, &texture_images, &iuv, &max_texture_height);
  PackWallTextures(wall_patches, FLAGS_texture_image_size,
                   &floorplan, &texture_images, &iuv, &max_texture_height);

  WriteTextureImages(file_io, FLAGS_texture_image_size, texture_images);
}
