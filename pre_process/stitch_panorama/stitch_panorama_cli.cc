#include <cstdlib>
#include <iostream>
#include <vector>
#include "stitch_panorama.h"
#include <Eigen/Dense>

using std::cerr;
using std::endl;
using std::vector;
using pre_process::Input;
using pre_process::StitchPanorama;

int main(int argc, char* argv[]) {
  if (argc < 5) {
    cerr << argv[0] << " directory width height num_levels" << endl;
    return 1;
  }

  const int width = atoi(argv[2]);
  const int height = atoi(argv[3]);
  const int num_levels = atoi(argv[4]);

  Input input;
  input.directory = argv[1];
  input.subsample = 5;

  vector<Eigen::Matrix3d> previous_rotations;
  for (int level = num_levels - 1; level >= 0; --level) {
    StitchPanorama stitch_panorama;
    input.level = level;
    input.out_width = width / (1 << level);
    input.out_height = height / (1 << level);
    // Critical parameters.
    input.margin = (int)round(0.05 * input.out_height);
    input.initial_rotations = previous_rotations;
  
    if (!stitch_panorama.Stitch(input)) {
      cerr << "Failed." << endl;
      return 1;
    }
    previous_rotations = stitch_panorama.GetRefinedRotations();

    //????
    break;
  }
  
  return 0;
}
