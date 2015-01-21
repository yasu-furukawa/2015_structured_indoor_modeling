#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <gflags/gflags.h>

#include "../base/kdtree/KDtree.h"
#include "../base/file_io.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "generate_texture.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

DEFINE_int32(start_panorama, 0, "First panorama ID.");
DEFINE_int32(num_pyramid_levels, 4, "Num pyramid levels.");
DEFINE_int32(max_num_rooms, 200, "Maximum number of rooms.");
// DEFINE_string(input_ply, "object_cloud_org.ply", "Input ply.");
// DEFINE_string(output_ply, "object_cloud2.ply", "Output ply.");

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

namespace {

  /*
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
  */

void FindVisiblePanoramas(const std::vector<std::vector<Panorama> >& panoramas,
                          const Point& point,
                          std::set<int>* visible_panoramas) {
  //  visible_panoramas->insert(4);
  // return;

  
  const int kLevel = 2;
  for (int p = 0; p < (int)panoramas.size(); ++p) {
    const Panorama& panorama = panoramas[p][kLevel];
    
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

  //????
  // use only a single pano.
    // visible_panoramas->clear();

  
  if (visible_panoramas->empty()) {
    const int kInvalid = -1;
    double closest_distance = 0.0;
    int closest_panorama = kInvalid;
    for (int p = 0; p < (int)panoramas.size(); ++p) {
      const Panorama& panorama = panoramas[p][kLevel];
      const Vector2d pixel = panorama.Project(point.position);
      const Vector3f rgb = panorama.GetRGB(pixel);
      if (rgb == Vector3f(0, 0, 0))
        continue;
      
      const double distance = (point.position - panoramas[p][kLevel].GetCenter()).norm();
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

  PointCloud point_cloud;
  ReadPointClouds(file_io, &point_cloud);

  const int kSkip = 4;
  vector<float> points;
  points.reserve(3 * point_cloud.GetNumPoints() / kSkip);

  for (int p = 0; p < point_cloud.GetNumPoints(); p += kSkip) {
    for (int i = 0; i < 3; ++i)
      points.push_back(point_cloud.GetPoint(p).position[i]);
  }

  cerr << "Build kdtree..." << flush;
  KDtree kdtree(points);
  cerr << "done." << endl;

  // Compute statistics.
  const int kNumNeighbors = 10;
  vector<const float*> knn;

  vector<float> neighbor_distances;
  for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
    knn.clear();
    const Vector3f ref_point(point_cloud.GetPoint(p).position[0],
                             point_cloud.GetPoint(p).position[1],
                             point_cloud.GetPoint(p).position[2]);
                      
    kdtree.find_k_closest_to_pt(knn, kNumNeighbors, &ref_point[0]);
                               
    double neighbor_distance = 0.0;
    for (int i = 0; i < kNumNeighbors; ++i) {
      const float* fp = knn[i];
      const Vector3f point(fp[0], fp[1], fp[2]);
      neighbor_distance += (point - ref_point).norm();
    }
    neighbor_distances.push_back(neighbor_distance / kNumNeighbors);
  }
  if (neighbor_distances.empty()) {
    cerr << "Impossible." << endl;
    exit (1);
  }

  double average = 0.0;
  for (int p = 0; p < neighbor_distances.size(); ++p)
    average += neighbor_distances[p];
  average /= neighbor_distances.size();
  double deviation = 0.0;
  for (int p = 0; p < neighbor_distances.size(); ++p) {
    const double diff = neighbor_distances[p] - average;
    deviation += diff * diff;
  }
  deviation /= neighbor_distances.size();
  deviation = sqrt(deviation);

  const double threshold = average + deviation;
  vector<Point> new_points;
  for (int p = 0; p < neighbor_distances.size(); ++p) {
    if (neighbor_distances[p] <= threshold) {
      new_points.push_back(point_cloud.GetPoint(p));
    }
  }
  point_cloud.SetPoints(new_points);


  
  const int kLevel = 2;
  for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
    Point& point = point_cloud.GetPoint(p);
    point.color = Vector3f(0, 0, 0);
    int denom = 0;
      
    set<int> visible_panoramas;
    FindVisiblePanoramas(panoramas, point, &visible_panoramas);
    if (visible_panoramas.empty()) {
      continue;
    }
    for (const auto panorama_id : visible_panoramas) {
      const Vector3f color =
        panoramas[panorama_id][kLevel].GetRGB(panoramas[panorama_id][kLevel].Project(point.position));
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
  point_cloud.Write(file_io.GetObjectPointCloudsWithColor());
  
  
  /*  
  ColoredPointCloud colored_point_cloud;
  {
    ReadPly(file_io.GetDataDirectory() + FLAGS_input_ply, &colored_point_cloud);
  }

  // Only use the color from 0 and 1 (if not black).
  const bool kUseSpecifiedPanoramas = false;
  if (kUseSpecifiedPanoramas) {
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
    
    WritePly(file_io.GetDataDirectory() + FLAGS_output_ply,
             colored_point_cloud);
  } else {
    const int kLevel = 0;
    
    ColoredPointCloud new_colored_point_cloud;
    for (auto& point : colored_point_cloud) {
      point.second = Vector3d(0, 0, 0);
      int denom = 0;
      
      set<int> visible_panoramas;
      FindVisiblePanoramas(panoramas, point.first, &visible_panoramas);
      if (visible_panoramas.empty()) {
        continue;
      }
      
      for (const auto panorama_id : visible_panoramas) {
        const Vector3f color =
          panoramas[panorama_id][kLevel].GetRGB(panoramas[panorama_id][kLevel].Project(point.first));
        if (color != Vector3f(0, 0, 0)) {
          for (int i = 0; i < 3; ++i)
            point.second[i] += color[i] / 255.0;
          ++denom;
        }
      }
      if (denom != 0) {
        point.second /= denom;
        
        new_colored_point_cloud.push_back(point);
      }
    }
    
    WritePly(file_io.GetDataDirectory() + FLAGS_output_ply,
             new_colored_point_cloud);
  }
  
  return 0;
  */
}
