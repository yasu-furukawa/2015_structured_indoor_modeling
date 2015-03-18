#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <gflags/gflags.h>

#ifdef _WIN32
#pragma comment (lib, "gflags.lib") 
#pragma comment (lib, "Shlwapi.lib") 
#endif

#include "../base/kdtree/KDtree.h"
#include "../base/file_io.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "generate_texture_floorplan.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

DEFINE_int32(num_pyramid_levels, 4, "Num pyramid levels.");
DEFINE_int32(max_num_rooms, 200, "Maximum number of rooms.");
DEFINE_int32(pyramid_level, 1, "Which level of pyramid to collect colors.");
// DEFINE_string(input_ply, "object_cloud_org.ply", "Input ply.");
// DEFINE_string(output_ply, "object_cloud2.ply", "Output ply.");

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

namespace {

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const Point& point,
                          const int pyramid_level,
                          std::set<int>* visible_panoramas) {
  //  visible_panoramas->insert(4);
  // return;

  // Use visible panoramas to average colors. (blurred and not good).
  /*  
  for (int p = 0; p < (int)panoramas.size(); ++p) {
    const Panorama& panorama = panoramas[p][pyramid_level];
    
    const Vector2d pixel = panorama.Project(point.position);
    const Vector3f rgb = panorama.GetRGB(pixel);
    if (rgb == Vector3f(0, 0, 0))
      continue;

    Vector3d point_to_camera = (panorama.GetCenter() - point.position).normalized();
    if (point_to_camera.dot(point.normal) <= 0.0)
      continue;
    
    const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
    const double depth = panorama.GetDepth(depth_pixel);
    const double dtmp = (point.position - panorama.GetCenter()).norm();
    const double margin = panorama.GetAverageDistance() / 100.0; // 20.0;
    if (dtmp < depth + margin) {
      visible_panoramas->insert(p);
    }
  }
  */

  
  // Use only a single pano. Sharp but needs to be careful about
  // stitching artifacts. Also, point density needs to be controlled.
  visible_panoramas->clear();
  
  if (visible_panoramas->empty()) {
    const int kInvalid = -1;
    double closest_distance = 0.0;
    int closest_panorama = kInvalid;
    for (int p = 0; p < (int)panoramas.size(); ++p) {
      const Panorama& panorama = panoramas[p][pyramid_level];
      const Vector2d pixel = panorama.Project(point.position);
      const Vector3f rgb = panorama.GetRGB(pixel);
      if (rgb == Vector3f(0, 0, 0))
        continue;
      
      const double distance = (point.position - panoramas[p][pyramid_level].GetCenter()).norm();
      if (distance < closest_distance || closest_panorama == kInvalid) {
        closest_distance = distance;
        closest_panorama = p;
      }
    }
    visible_panoramas->insert(closest_panorama);
  }
}

void ReadPointClouds(const FileIO& file_io,
                     PointCloud* point_cloud) {
  for (int room = 0; room < FLAGS_max_num_rooms; ++room) {
    PointCloud pc;
    if (pc.Init(file_io.GetObjectPointClouds(room))) {
      point_cloud->AddPoints(pc);
    }
  }
}

}  // namespace

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
  FileIO file_io(argv[1]);

  vector<vector<Panorama> > panoramas;
  {
    ReadPanoramaPyramids(file_io, FLAGS_num_pyramid_levels, &panoramas);
  }
  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramaToGlobals(file_io, &panorama_to_globals);
  }
  vector<Matrix4d> global_to_panoramas;
  {
    Invert(panorama_to_globals, &global_to_panoramas);
  }

  PointCloud point_cloud;
  ReadPointClouds(file_io, &point_cloud);

  for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
    if (p % (point_cloud.GetNumPoints() / 100) == 0)
      cerr << '.' << flush;
    Point& point = point_cloud.GetPoint(p);
    point.color = Vector3f(0, 0, 0);
    int denom = 0;
      
    set<int> visible_panoramas;
    FindVisiblePanoramas(panoramas, point, FLAGS_pyramid_level, &visible_panoramas);
    if (visible_panoramas.empty()) {
      continue;
    }
    for (const auto panorama_id : visible_panoramas) {
      const Vector3f color =
        panoramas[panorama_id][FLAGS_pyramid_level].GetRGB(panoramas[panorama_id][FLAGS_pyramid_level].Project(point.position));
      if (color != Vector3f(0, 0, 0)) {
        for (int i = 0; i < 3; ++i)
          point.color[i] += color[2 - i];
        ++denom;
      }
    }
    if (denom != 0) {
      point.color /= denom;
    }
  }
  cerr << "done." << endl;
  point_cloud.Write(file_io.GetObjectPointCloudsWithColor());
}
