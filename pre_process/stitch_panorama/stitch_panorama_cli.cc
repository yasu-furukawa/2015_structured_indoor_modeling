#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include "stitch_panorama.h"
#include <Eigen/Dense>

using std::cerr;
using std::endl;
using std::ofstream;
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
  input.subsample = 1;

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

    char buffer[1024];
    sprintf(buffer, "%s/IMU_rotation_new_%d.txt", input.directory.c_str(), level);
    ofstream ofstr;
    ofstr.open(buffer);
    if (!ofstr.is_open()) {
      cerr << "No IMU_rotation file." << endl;
      return 1;
    }
    ofstr << previous_rotations.size() << endl;
    for (const auto& mat : previous_rotations) {
      for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
          ofstr << mat(x, y) << ' ';
        }
      }
      ofstr << endl;
    }
    ofstr.close();
  }

  return 0;
}
