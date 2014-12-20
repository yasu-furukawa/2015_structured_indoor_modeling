#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "../submodular/data.h"
#include "../submodular/evidence.h"
#include "../submodular/transform.h"
#include "door_detection.h"
// #include "preprocess.h"
// #include "reconstruct_2d.h"

using namespace Eigen;
using namespace std;
using namespace floored;
using namespace room_segmentation;

namespace {

void ReadSweeps(const string directory, const string file_prefix,
                vector<Sweep>* sweeps) {
  sweeps->clear();
  cerr << "Reading points... " << flush;

  const int kMaxNum = 200;
  for (int s = 0; s < kMaxNum; ++s) {
    char buffer[1024];
    sprintf(buffer, "%s%s%03d.ply", directory.c_str(), file_prefix.c_str(), s);

    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open())
      continue;

    cerr << buffer << endl;

    ply::Points points;
    ifstr >> points;
    ifstr.close();


    //?????
    // Autodesk
    /*
    {
      ply::Points points_tmp;
      points_tmp.push_back(points[0]);

      for (int i = 1; i < points.size(); ++i) {
        if ((points[i].position - points[0].position).norm() < 140)
          points_tmp.push_back(points[i]);
      }
      points_tmp.swap(points);
    
      random_shuffle(points.begin() + 1, points.end());
      points.resize(points.size() * 0.1);


      char buffer[1024];
      sprintf(buffer, "subsampled_%03d.ply", s);
      ofstream ofstr;
      ofstr.open(buffer);

      ofstr << "ply" << endl
            << "format ascii 1.0" << endl
            << "element vertex " << points.size() << endl
            << "property float x" << endl
            << "property float y" << endl
            << "property float z" << endl
            << "end_header" << endl;

      for (const auto& point : points) {
        ofstr << point.position[0] << ' '
              << point.position[1] << ' '
              << point.position[2] << endl;
      }
      ofstr.close();
    }
    */
    

    // Convert poitns to sweeps, assuming that the first point is the center.
    Sweep sweep;
    ConvertPointsToSweep(points, &sweep);
    sweeps->push_back(sweep);
  }
 
  cerr << endl;
}
  
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
    cout << "compute" << endl;
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
  if (argc < 3) {
    cerr << "Usage: " << argv[0]
         << " directory_prefix ply_prefix skip_reading" << endl
         << "./room_segmentation ../data/mobile/ transformed_0" << endl;
    return 1;
  }

  bool skip_reading = false;
  if (argc >= 4 && atoi(argv[3]))
    skip_reading = true;
  
  vector<Sweep> sweeps;
  if (!skip_reading)
    ReadSweeps(argv[1], argv[2], &sweeps);
  const float average_distance = ComputeAverageDistance(sweeps);
  
  Frame frame;
  InitializeFrame(argv[1], sweeps, average_distance, &frame);

  ConvertSweepsToFrame(frame, &sweeps);
  
  vector<float> point_evidence, free_space_evidence;
  InitializeEvidence(sweeps, frame, argv[1], &point_evidence, &free_space_evidence);

  vector<float> door_detection;
  DetectDoors(sweeps, frame, argv[1], point_evidence, free_space_evidence, &door_detection);

  

  
  return 0;
}
