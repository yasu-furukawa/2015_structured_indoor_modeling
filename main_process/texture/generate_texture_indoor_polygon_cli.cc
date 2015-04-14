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
#include "generate_texture.h"
#include "generate_texture_indoor_polygon.h"

DEFINE_int32(start_panorama, 0, "First panorama id.");
DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
DEFINE_int32(pyramid_level, 1, "Level of pyramid for floor texture.");
DEFINE_int32(max_texture_size_per_non_floor_patch, 1024, "Maximum texture size for each wall patch.");
DEFINE_int32(target_texture_size_for_vertical, 250, "How many texture pixels between floor and ceiling.");

DEFINE_double(position_error_for_floor, 0.08, "How much error is allowed for a point to be on a floor.");

// The following flags should be rescaled together. They are sensitive.
DEFINE_int32(max_texture_size_per_floor_patch, 1500, "Maximum texture size for each floor patch.");
DEFINE_int32(patch_size_for_synthesis, 45, "Patch size for synthesis."); // 45
DEFINE_int32(num_cg_iterations, 40, "Number of CG iterations.");

DEFINE_int32(texture_image_size, 2048, "Texture image size to be written.");

DEFINE_string(binary_ply, "", "A file name under directory.");
DEFINE_string(ascii_ply, "", "A file name under directory.");

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

string ExtractSuffix(const string filename) {
  return filename.substr(filename.find_last_of('/') + 1,
                         filename.length());
  // filename.find_last_of('.'));
}

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

  // Read data from the directory.
  FileIO file_io(argv[1]);

  TextureInput texture_input;
  {
    ReadPanoramaPyramids(file_io, FLAGS_num_pyramid_levels, &texture_input.panoramas);
  }
  
  {
    ReadPointClouds(file_io, &texture_input.point_clouds);
  }
  
  if (FLAGS_binary_ply == "" && FLAGS_ascii_ply == "") {
    const string filename = file_io.GetIndoorPolygon();
    ifstream ifstr;
    ifstr.open(filename.c_str());
    ifstr >> texture_input.indoor_polygon;
    ifstr.close();

    texture_input.num_patch_half_iterations = 3;
    texture_input.erode_texture = true;
  } else if (FLAGS_binary_ply != "") {
    char buffer[1024];
    sprintf(buffer, "%s%s", argv[1], FLAGS_binary_ply.c_str());
    texture_input.indoor_polygon.InitFromBinaryPly(buffer);

    texture_input.num_patch_half_iterations = 12;
    texture_input.erode_texture = false;
  } else if (FLAGS_ascii_ply != "") {
    char buffer[1024];
    sprintf(buffer, "%s%s", argv[1], FLAGS_ascii_ply.c_str());
    texture_input.indoor_polygon.InitFromAsciiPly(buffer);

    texture_input.num_patch_half_iterations = 3;
    texture_input.erode_texture = true;
  } else {
    cerr << "Impossible." << endl;
    exit (1);
  }

  {
    texture_input.pyramid_level            = FLAGS_pyramid_level;
    texture_input.max_texture_size_per_floor_patch     = FLAGS_max_texture_size_per_floor_patch;
    texture_input.max_texture_size_per_non_floor_patch = FLAGS_max_texture_size_per_non_floor_patch;
    texture_input.position_error_for_floor = FLAGS_position_error_for_floor;
    texture_input.patch_size_for_synthesis = FLAGS_patch_size_for_synthesis;
    texture_input.num_cg_iterations        = FLAGS_num_cg_iterations;
  }
  texture_input.texel_unit =
    ComputeTexelUnit(texture_input.indoor_polygon, FLAGS_target_texture_size_for_vertical);
  const double default_visibility_margin = ComputeVisibilityMargin(texture_input.indoor_polygon);

  vector<Patch> patches(texture_input.indoor_polygon.GetNumSegments());

  for (int p = 0; p < patches.size(); ++p) {
    const Segment& segment = texture_input.indoor_polygon.GetSegment(p);

    bool visibility_check;
    if (segment.type == Segment::FLOOR) {
      visibility_check = true;
      texture_input.visibility_margin = default_visibility_margin / 2;
    } else {
      // visibility_check = false;
      visibility_check = true;
      texture_input.visibility_margin = default_visibility_margin;
    }
    SetPatch(texture_input, segment, visibility_check, &patches[p]);
  }

  // Texture image.
  vector<vector<unsigned char> > texture_images;
  // Texture coordinate.
  pair<int, Vector2i> iuv(0, Vector2i(0, 0));
  int max_texture_height = 0;

  for (int p = 0; p < patches.size(); ++p) {
    PackTexture(patches[p],
                FLAGS_texture_image_size,
                &texture_input.indoor_polygon.GetSegment(p),
                &texture_images,
                &iuv,
                &max_texture_height);
  }

  string suffix("");
  if (FLAGS_binary_ply != "")
    suffix = ExtractSuffix(FLAGS_binary_ply);
  else if (FLAGS_ascii_ply != "")
    suffix = ExtractSuffix(FLAGS_ascii_ply);
  
  WriteTextureImages(file_io, FLAGS_texture_image_size, texture_images, suffix);
  {
    ofstream ofstr;
    ofstr.open(file_io.GetIndoorPolygonFinal(suffix).c_str());
    ofstr << texture_input.indoor_polygon;
    ofstr.close();
  }

  return 0;
}
