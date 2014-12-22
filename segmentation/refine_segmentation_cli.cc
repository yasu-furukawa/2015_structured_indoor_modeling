#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "../base/mrf/GCoptimization.h"
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
    sprintf(buffer, "%s/frame.dat", directory.c_str());
    ifstream ifstr;
    ifstr.open(buffer);
    ifstr >> frame;
    ifstr.close();
  }
  
  vector<float> point_evidence, free_space_evidence;
  {
    const string point_evidence_filename = directory + "point_evidence.dat";
    const string free_space_evidence_filename = directory + "free_space_evidence.dat";
    LoadEvidence(point_evidence_filename, &point_evidence);
    LoadEvidence(free_space_evidence_filename, &free_space_evidence);
  }

  vector<Vector2i> centers;
  vector<vector<Vector2i> > clusters;
  {
    const string cluster_file = directory + "initial_cluster.dat";
    LoadClustering(cluster_file, &centers, &clusters);
  }

  
  
  

  return 0;
}
