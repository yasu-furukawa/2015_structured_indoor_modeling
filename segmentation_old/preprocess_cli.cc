#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "data.h"
#include "evidence.h"
#include "preprocess.h"
#include "reconstruct_2d.h"
#include "transform.h"

using namespace Eigen;
using namespace std;
using namespace floored;

namespace {

void InitializeFrame(const string directory, const std::vector<Sweep>& sweeps,
                     const float average_distance, Frame* frame) {
  char buffer[1024];
  sprintf(buffer, "%s/frame.dat", directory.c_str());
  ifstream ifstr;
  ifstr.open(buffer);
  if (ifstr.is_open()) {
    ifstr >> *frame;
    ifstr.close();
  } else {
    ifstr.close();
    ComputeFrame(directory, sweeps, average_distance, frame);
    
    ofstream ofstr;
    ofstr.open(buffer);
    ofstr << *frame;
    ofstr.close();
  }
}
  
void InitializeEvidence(const std::vector<Sweep>& sweeps,
                        const Frame& frame,
                        const string directory,
                        vector<float>* point_evidence,
                        vector<float>* free_space_evidence) {
  const string point_evidence_filename = directory + "point_evidence.dat";
  const string free_space_evidence_filename = directory + "free_space_evidence.dat";
  ifstream ifstr0, ifstr1;
  ifstr0.open(point_evidence_filename.c_str());
  ifstr1.open(free_space_evidence_filename.c_str());

  if (ifstr0.is_open() && ifstr1.is_open()) {
    ifstr0.close();
    ifstr1.close();
    cerr << "Loading evidence..." << flush;
    LoadEvidence(point_evidence_filename, point_evidence);
    LoadEvidence(free_space_evidence_filename, free_space_evidence);

    DrawEvidenceToImage(frame, directory, *point_evidence, *free_space_evidence);
    
    cerr << "done." << endl;
  } else {
    ifstr0.close();
    ifstr1.close();
    cerr << "Computing evidence..." << flush;
    SetPointEvidence(sweeps, frame, directory, point_evidence);
    SetFreeSpaceEvidence(sweeps, frame, directory, free_space_evidence);

    DrawEvidenceToImage(frame, directory, *point_evidence, *free_space_evidence);
    
    cerr << "done." << endl;
    WriteEvidence(point_evidence_filename, *point_evidence);
    WriteEvidence(free_space_evidence_filename, *free_space_evidence);
  }
}

}  // namespace

//----------------------------------------------------------------------
int main(int argc, char* argv[]) {
  /*
  if (argc < 2) {
    cerr << "Usage: " << argv[0]
         << " prefix"
	 << endl;
    return 1;
  }

  // Loads frame.
  vector<Sweep> sweeps;
  float average_distance = 0.0;
  
  Frame frame;
  InitializeFrame(argv[1], sweeps, average_distance, &frame);

  vector<float> point_evidence, free_space_evidence;
  InitializeEvidence(sweeps, frame, argv[1], &point_evidence, &free_space_evidence);

  Floorplan floorplan;
  Reconstruct2D(frame, point_evidence, free_space_evidence, argv[1], &floorplan);
  
  return 0;
  */

  if (argc < 4) {
    cerr << "Usage: " << argv[0]
         << " prefix num_of_sweeps reference_sweep_id"
	 << endl;
    return 1;
  }


  const int num = atoi(argv[2]);
  const int reference_num = atoi(argv[3]);

  // Transforms from each sweep to the reference camera.
  vector<Matrix4f> transforms;
  ReadTransforms(argv[1], num, reference_num, &transforms);

  // Read point data and compute normal.
  vector<Sweep> sweeps;
  ReadSweeps(argv[1], num, &sweeps);
  const float average_distance = ComputeAverageDistance(sweeps);

  // Transform points.
  TransformSweeps(transforms, &sweeps);
  {
    char buffer[1024];
    sprintf(buffer, "%s/sweeps.ply", argv[1]);
    WriteSweepsAsPly(sweeps, buffer);
  }
  
  // Computes frame.
  Frame frame;
  InitializeFrame(argv[1], sweeps, average_distance, &frame);

  // Convert to the cannical coordinate frame.
  ConvertSweepsToFrame(frame, &sweeps);

  // Only keep sweeps at certain height.
  {
    // const double lower_height = 0.6 * frame.size[2];
    // const double upper_height = 0.8 * frame.size[2];
    // FilterSweeps(lower_height, upper_height, &sweeps);
  }
  {
    char buffer[1024];
    sprintf(buffer, "%s/rotated_sweeps.ply", argv[1]);
    WriteSweepsAsPly(sweeps, buffer);
  }

  vector<float> point_evidence, free_space_evidence;
  InitializeEvidence(sweeps, frame, argv[1], &point_evidence, &free_space_evidence);

  Floorplan floorplan;
  Reconstruct2D(frame, point_evidence, free_space_evidence, argv[1], &floorplan);
  
  return 0;
}
