#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "data.h"
#include "segmentation.h"
#include "../calibration/file_io.h"

using namespace Eigen;
using namespace segmentation;
using namespace std;

const double kMinSigmaIntensity = -2.5;
const double kMaxSigmaIntensity = 1.0;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << "data_directory [recompute] [rotation_angle_degrees]" << endl;
    exit (1);
  }

  file_io::FileIO file_io(argv[1]);
  vector<Sweep> sweeps;
  ReadSweeps(file_io, &sweeps);
  for (auto& sweep : sweeps) {
    NormalizeIntensity(kMinSigmaIntensity, kMaxSigmaIntensity, &sweep);
  }
  if (argc > 4) {
    const double angle = atof(argv[3]) * M_PI / 180.0;
    for (auto& sweep : sweeps) {
      RotateSweep(angle, &sweep);
    }
  }

  bool recompute = false;
  if (argc < 3) {
    if (atoi(argv[2]))
      recompute = true;
  }

  const float average_distance = ComputeAverageDistance(sweeps);
  cout << average_distance << endl;
  
  Frame frame;
  InitializeFrame(recompute, argv[1], sweeps, average_distance, &frame);
  
  vector<float> point_evidence, free_space_evidence;
  vector<Eigen::Vector3d> normal_evidence;  
  InitializeEvidence(recompute, sweeps, frame, argv[1],
                     &point_evidence,
                     &free_space_evidence,
                     &normal_evidence);

  //  do real compu
  
  return 0;
}
