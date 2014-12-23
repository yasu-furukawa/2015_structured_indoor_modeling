#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "refine_segmentation.h"
#include "door_detection.h"
#include "evidence.h"

using namespace Eigen;
using namespace std;
using namespace floored;
using namespace room_segmentation;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0]
         << " directory_prefix" << endl
         << "./room_segmentation ../data/mobile/" << endl;
    return 1;
  }

  const string directory = argv[1];
  
  Frame frame;
  {
    char buffer[1024];
    sprintf(buffer, "%sframe.dat", directory.c_str());
    ifstream ifstr;
    ifstr.open(buffer);
    ifstr >> frame;
    ifstr.close();
  }
  
  vector<float> point_evidence, free_space_evidence;
  vector<Vector3d> normal_evidence;
  {
    const string point_evidence_filename = directory + "point_evidence.dat";
    const string free_space_evidence_filename = directory + "free_space_evidence.dat";
    const string normal_evidence_filename = directory + "normal_evidence.dat";
    LoadEvidence(point_evidence_filename, &point_evidence);
    LoadEvidence(free_space_evidence_filename, &free_space_evidence);
    LoadEvidence3(normal_evidence_filename, &normal_evidence);
    {
      const double kMinSigma = -0.3;
      const double kMaxSigma = 0.0;
      NormalizeEvidence(kMinSigma, kMaxSigma, &point_evidence);
    }
    {
      const double kMinSigma = -1.0;
      const double kMaxSigma = 0.2;
      NormalizeEvidence(kMinSigma, kMaxSigma, &free_space_evidence);
    }
    const double kScale = 255.0;
    const string point_image = directory + "point_normalized.ppm";
    DrawEvidence(frame.size[0], frame.size[1], point_evidence, point_image, kScale);
    const string free_space_image = directory + "free_space_normalized.ppm";
    DrawEvidence(frame.size[0], frame.size[1], free_space_evidence, free_space_image, kScale);
    const string normal_image = directory + "normal_normalized.ppm";
    DrawEvidence3(frame.size[0], frame.size[1], frame.axes[0], frame.axes[1], normal_evidence, normal_image);
  }

  vector<Vector2i> centers;
  vector<vector<Vector2i> > clusters;
  {
    const string cluster_file = directory + "initial_cluster.dat";
    LoadClustering(cluster_file, &centers, &clusters);
  }
  vector<vector<pair<int, float> > > visibility;
  {
    const string visibility_file = directory + "visibility.dat";
    int width, height, subsample;
    LoadVisibility(visibility_file, &visibility, &width, &height, &subsample);
    if (width != frame.size[0] || height != frame.size[1]) {
      cerr << "Incompatible dimension: " << width << ' ' << height << ' ' << frame.size[0] << ' ' << frame.size[1] << endl;
      exit (1);
    }
    ExpandVisibility(width, height, &visibility);
  }

  vector<int> segmentation;
  RefineSegmentation(frame, point_evidence, free_space_evidence, normal_evidence,
                     centers, clusters, visibility, &segmentation);

  {
    const string output_file = directory + "segmentation.ppm";
    WriteSegmentation(output_file, segmentation, frame.size[0], frame.size[1], centers.size() + 1);
  }

  return 0;
}
