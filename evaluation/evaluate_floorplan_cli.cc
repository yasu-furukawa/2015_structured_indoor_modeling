#include <iostream>
#include <fstream>
#include <vector>

#include <gflags/gflags.h>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/point_cloud.h"

DEFINE_string(floorplan_file, "", "Floorplan filename.");

using namespace structured_indoor_modeling;
using namespace std;

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
  
  string floorplan_file;
  if (FLAGS_floorplan_file == "") {
    floorplan_file = file_io.GetFloorplan();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", argv[1], FLAGS_floorplan_file.c_str());
    floorplan_file = buffer;
  }
    
  Floorplan floorplan;
  {
    ifstream ifstr;
    ifstr.open(floorplan_file.c_str());
    ifstr >> floorplan;
    ifstr.close();
  }

  vector<PointCloud> point_clouds(end_panorama);
  {
    cout << "Reading point clouds..." << flush;
    for (int p = 0; p < end_panorama; ++p) {
      cout << '.' << flush;
      const int index = p;
      if (!point_clouds[index].Init(file_io, p)) {
        cerr << "Failed in loading the point cloud." << endl;
        exit (1);
      }
      // Make the 3D coordinates into the floorplan coordinate system.
      point_clouds[index].ToGlobal(file_io, p);
      const Matrix3d global_to_floorplan = floorplan.GetFloorplanToGlobal().transpose();
      point_clouds[index].Rotate(global_to_floorplan);
    }
  }
  // Evaluate.
  


  return 0;
}
