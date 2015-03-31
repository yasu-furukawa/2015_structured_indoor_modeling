#include <iostream>
#include <fstream>
#include <limits>
#include <vector>

#include <gflags/gflags.h>

#include "../../base/file_io.h"
#include "../../base/point_cloud.h"

#ifdef _WIN32
#pragma comment (lib, "gflags.lib") 
#pragma comment (lib, "Shlwapi.lib") 
#endif

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

  FileIO file_io(argv[1]);
  const int num_panoramas = GetNumPanoramas(file_io);
  vector<vector<double> > oriented_points;
  for (int panorama = 0; panorama < num_panoramas; ++panorama) {
    PointCloud point_cloud;
    point_cloud.Init(file_io, panorama);
    point_cloud.ToGlobal(file_io, panorama);
    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Point& point = point_cloud.GetPoint(p);
      vector<double> oriented_point;
      for (int i = 0; i < 3; ++i)
        oriented_point.push_back(point.position[i]);
      for (int i = 0; i < 3; ++i)
        oriented_point.push_back(point.normal[i]);
      oriented_points.push_back(oriented_point);
    }
  }

  ofstream ofstr;
  ofstr.open(file_io.GetPoissonInput().c_str());
  for (const auto& oriented_point : oriented_points) {
    for (const auto& value : oriented_point)
      ofstr << value << ' ';
    ofstr << endl;
  }
  ofstr.close();  
  
  return 0;
}
