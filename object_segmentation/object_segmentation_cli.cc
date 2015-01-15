#include <fstream>
#include <gflags/gflags.h>
#include <iostream>

#include "../base/floorplan.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace structured_indoor_modeling;
using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }

  Floorplan floorplan;
  floorplan

}
