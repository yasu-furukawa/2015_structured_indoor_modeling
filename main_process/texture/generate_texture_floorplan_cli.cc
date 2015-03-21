#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <gflags/gflags.h>

#ifdef _WIN32
#pragma comment (lib, "gflags.lib") 
#pragma comment (lib, "Shlwapi.lib") 
#endif

#include "../../base/file_io.h"
#include "../../base/floorplan.h"
#include "../../base/panorama.h"
#include "generate_texture_floorplan.h"

DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
DEFINE_int32(pyramid_level_for_floor, 1, "Level of pyramid for floor texture.");
// DEFINE_double(texel_size_rescale, 1.0, "Less than 1 to increase resolution.");
DEFINE_int32(max_texture_size_per_wall_patch, 1024, "Maximum texture size for each wall patch.");
DEFINE_int32(texture_height_per_wall, 512, "Texture height for each wall patch.");
DEFINE_int32(texture_image_size, 2048, "Texture image size to be written.");

DEFINE_double(position_error_for_floor, 0.08, "How much error is allowed for a point to be on a floor.");

// The following flags should be rescaled together. They are sensitive.
DEFINE_int32(max_texture_size_per_floor_patch, 1500, "Maximum texture size for each floor patch.");
DEFINE_int32(patch_size_for_synthesis, 45, "Patch size for synthesis.");
DEFINE_int32(num_cg_iterations, 40, "Number of CG iterations.");

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
#ifdef __APPLE__
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif
  // google::InitGoogleLogging(argv[0]);

  // Read data from the directory.
  FileIO file_io(argv[1]);

  TextureInput texture_input;
  {
    ReadPanoramaPyramids(file_io,
                         FLAGS_num_pyramid_levels,
                         &texture_input.panoramas);
  }
  {
    ReadPointClouds(file_io, &texture_input.point_clouds);
  }
  {
    const string filename = file_io.GetFloorplan();
    ifstream ifstr;
    ifstr.open(filename.c_str());
    ifstr >> texture_input.floorplan;
    ifstr.close();
  }
  {
    texture_input.pyramid_level_for_floor = FLAGS_pyramid_level_for_floor;
    texture_input.max_texture_size_per_floor_patch = FLAGS_max_texture_size_per_floor_patch;
    texture_input.max_texture_size_per_wall_patch = FLAGS_max_texture_size_per_wall_patch;
    texture_input.texture_height_per_wall  = FLAGS_texture_height_per_wall;
    texture_input.position_error_for_floor = FLAGS_position_error_for_floor;
    texture_input.patch_size_for_synthesis = FLAGS_patch_size_for_synthesis;
    texture_input.num_cg_iterations        = FLAGS_num_cg_iterations;
  }
  // Unit for a texel.
  // const double texel_size = ComputeTexelSize(panoramas) * FLAGS_texel_size_rescale;
  
  // For each wall rectangle, floor, and ceiling,
  // 0. Identify visible panoramas.
  // 1. Grab texture
  // 2. Stitch
  // Floor texture.
  cerr << "Set floor patch." << endl;
  Patch floor_patch;
  SetFloorPatch(texture_input, &floor_patch);
  cerr << "done." << endl << "Set wall patches." << endl;
  // Wall textures.
  vector<vector<Patch> > wall_patches;
  SetWallPatches(texture_input, &wall_patches);
  cerr << "done." << endl << "Pack textures." << endl;
  // Texture image.
  vector<vector<unsigned char> > texture_images;
  // Texture coordinate.
  pair<int, Vector2i> iuv(0, Vector2i(0, 0));
  int max_texture_height = 0;

  // Set texture coordinates.
  PackFloorTexture(floor_patch, FLAGS_texture_image_size,
                   &texture_input.floorplan, &texture_images, &iuv, &max_texture_height);
  PackWallTextures(wall_patches, FLAGS_texture_image_size,
                   &texture_input.floorplan, &texture_images, &iuv, &max_texture_height);
  cerr << "done." << endl;
  WriteTextureImages(file_io, FLAGS_texture_image_size, texture_images);
  {
    ofstream ofstr;
    ofstr.open(file_io.GetFloorplanFinal().c_str());
    ofstr << texture_input.floorplan;
    ofstr.close();
  }
}
