#include <iostream>
#include <fstream>
#include <vector>

#include <gflags/gflags.h>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "evaluate.h"

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
    
  Floorplan floorplan(floorplan_file);

  vector<Panorama> panoramas;
  ReadPanoramas(file_io, &panoramas);

  vector<PointCloud> input_point_clouds, object_point_clouds;
  ReadInputPointClouds(file_io, &input_point_clouds);
  ReadObjectPointClouds(file_io, floorplan.GetNumRooms(), &object_point_clouds);

  // Accuracy and completeness.
  //
  // Rasterize from the floorplan data, input_point_clouds
  for (int p = 0; p < (int)input_point_clouds.size(); ++p) {
    

  
  
  }

  return 0;
}
