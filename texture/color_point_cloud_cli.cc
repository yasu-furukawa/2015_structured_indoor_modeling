#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <gflags/gflags.h>

#include "../base/file_io.h"
#include "../base/panorama.h"
#include "generate_texture.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

DEFINE_int32(start_panorama, 0, "First panorama ID.");
DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
DEFINE_string(input_ply, "object_cloud.ply", "Input ply.");
DEFINE_string(output_ply, "object_cloud2.ply", "Output ply.");

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

namespace {

void ReadPly(const string filename, ColoredPointCloud* colored_point_cloud) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  string header;
  for (int i = 0; i < 6; ++i)
    ifstr >> header;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 22; ++i)
    ifstr >> header;

  colored_point_cloud->resize(num_points);
  for (int p = 0; p < num_points; ++p) {
    for (int i = 0; i < 3; ++i)
      ifstr >> colored_point_cloud->at(p).first[i];
    for (int i = 0; i < 3; ++i) {
      ifstr >> colored_point_cloud->at(p).second[i];
      colored_point_cloud->at(p).second[i] /= 255.0;
    }
    double dtmp;
    ifstr >> dtmp;
  }
  ifstr.close();
}

void WritePly(const string filename, const ColoredPointCloud& colored_point_cloud) {
  ofstream ofstr;
  ofstr.open(filename.c_str());

  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "element vertex " << colored_point_cloud.size() << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property uchar red" << endl
        << "property uchar green" << endl
        << "property uchar blue" << endl
        << "property uchar alpha" << endl
        << "end_header" << endl;

  for (const auto& colored_point : colored_point_cloud) {
    ofstr << colored_point.first[0] << ' '
          << colored_point.first[1] << ' '
          << colored_point.first[2] << ' '
          << static_cast<int>(round(255.0 * colored_point.second[2])) << ' '
          << static_cast<int>(round(255.0 * colored_point.second[1])) << ' '
          << static_cast<int>(round(255.0 * colored_point.second[0])) << ' '
          << 255 << endl;
  }

  ofstr.close();
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);

  FileIO file_io(argv[1]);
  const int end_panorama = GetEndPanorama(file_io, FLAGS_start_panorama);

  vector<vector<Panorama> > panoramas;
  {
    ReadPanoramas(file_io,
                  FLAGS_start_panorama,
                  end_panorama,
                  FLAGS_num_pyramid_levels,
                  &panoramas);
  }
  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramaToGlobals(file_io, FLAGS_start_panorama, end_panorama, &panorama_to_globals);
  }
  vector<Matrix4d> global_to_panoramas;
  {
    Invert(panorama_to_globals, &global_to_panoramas);
  }
  ColoredPointCloud colored_point_cloud;
  {
    ReadPly(file_io.GetDataDirectory() + FLAGS_input_ply, &colored_point_cloud);
  }

  // Only use the color from 0 and 1 (if not black).
  vector<int> panorama_ids;
  // panorama_ids.push_back(0);
  // panorama_ids.push_back(1);
  panorama_ids.push_back(2);
  const int kLevel = 0;

  for (auto& point : colored_point_cloud) {
    point.second = Vector3d(0, 0, 0);
    int denom = 0;
    for (const auto panorama_id : panorama_ids) {
      const Vector3f color =
        panoramas[panorama_id][kLevel].GetRGB(panoramas[panorama_id][kLevel].Project(point.first));
      if (color != Vector3f(0, 0, 0)) {
        for (int i = 0; i < 3; ++i)
          point.second[i] += color[i] / 255.0;
        ++denom;
      }
    }
    if (denom != 0)
      point.second /= denom;
  }

  WritePly(file_io.GetDataDirectory() + FLAGS_output_ply, colored_point_cloud);

  return 0;
}
