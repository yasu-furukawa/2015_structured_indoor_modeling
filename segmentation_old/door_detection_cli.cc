#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "data.h"
#include "evidence.h"
#include "transform.h"
#include "door_detection.h"
// #include "preprocess.h"
// #include "reconstruct_2d.h"

using namespace Eigen;
using namespace std;
using namespace floored;
using namespace room_segmentation;

namespace {

void NormalizeIntensity(ply::Points* points) {
  double average_intensity = 0.0;
  int denom = 0;
  for (const auto& point : *points) {
    if (point.intensity != 0.0) {
      average_intensity += point.intensity;
      ++denom;
    }
  }

  if (denom == 0.0) {
    cerr << "No non-zero intensity point?" << endl;
    exit (1);
  }
  average_intensity /= denom;
  double deviation = 0.0;
  for (const auto& point : *points) {
    if (point.intensity != 0.0) {
      deviation += (point.intensity - average_intensity) *
        (point.intensity - average_intensity);
    }
  }
  deviation /= denom;
  deviation = sqrt(deviation);

  for (auto& point : *points) {
    point.intensity =
      (point.intensity - average_intensity) / deviation;
    point.intensity = 255 * max(0.0, min(1.0, (point.intensity + 1.0) / 1.0));
  }

  //----------------------------------------------------------------------
  ply::Points points_tmp = *points;
  points->clear();
  for (const auto& point : points_tmp) {
    if (point.intensity != 0.0)
      points->push_back(point);
  }
}

void ReadSweeps(const string directory, vector<Sweep>* sweeps) {
  sweeps->clear();
  cerr << "Reading points... " << flush;

  const int kMaxNum = 200;
  for (int s = 0; s < kMaxNum; ++s) {
    char buffer[1024];
    sprintf(buffer, "%sply/%03d.ply", directory.c_str(), s + 1);

    ifstream ifstr;
    ifstr.open(buffer);
    if (!ifstr.is_open())
      continue;

    cerr << buffer << endl;

    ifstream ifstr2;
    sprintf(buffer, "%stransformations/%03d.txt", directory.c_str(), s + 1);
    ifstr2.open(buffer);
    Matrix4d transformation;
    char ctmp;
    ifstr2 >> ctmp;
    for (int y = 0; y < 3; ++y)
      for (int x = 0; x < 4; ++x)
        ifstr2 >> transformation(y, x);
    transformation(3, 0) = 0;
    transformation(3, 1) = 0;
    transformation(3, 2) = 0;
    transformation(3, 3) = 1;
    ifstr2.close();

    /*
    {
      ifstream ifstr3;
      sprintf(buffer, "%srotationmat.txt", directory.c_str());
      ifstr3.open(buffer);
      Matrix4d rot;
      for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
          rot(y, x) = 0;
      for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x)
          ifstr3 >> rot(y, x);
      rot(3, 3) = 1;

      transformation = rot.transpose() * transformation;

      ifstr3.close();
    }
    */
    {
      ifstream ifstr3;
      sprintf(buffer, "%sfloorplan.txt", directory.c_str());
      ifstr3.open(buffer);
      Matrix4d rot;
      for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
          rot(y, x) = 0;
      for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x)
          ifstr3 >> rot(y, x);
      rot(3, 3) = 1;

      transformation = rot.transpose() * transformation;

      ifstr3.close();
    }

    
    ply::Points points;
    {
      string header;
      for (int i = 0; i < 6; ++i)
        ifstr >> header;
      int num_points;
      ifstr >> num_points;
      for (int i = 0; i < 37; ++i)
        ifstr >> header;

      ply::Point point;
      point.position = Vector3f(transformation(0, 3),
                                transformation(1, 3),
                                transformation(2, 3));
      point.normal = Vector3f(0, 0, 0);
      point.color = Vector3f(0, 0, 0);
                                
      points.push_back(point);
      
      for (int p = 0; p < num_points; ++p) {
        int itmp;
        ifstr >> itmp >> itmp;
        Vector4d local;
        for (int i = 0; i < 3; ++i)
          ifstr >> local(i);
        local(3) = 1.0;
        Vector4d global = transformation * local;

        ply::Point point;
        for (int i = 0; i < 3; ++i)
          point.position(i) = global(i);

        for (int i = 0; i < 3; ++i) {
          ifstr >> point.color[i];
          point.color[i] /= 255.0;
        }
        
        Vector4d local_normal;
        for (int i = 0; i < 3; ++i)
          ifstr >> local_normal(i);
        local_normal(3) = 0.0;

        //????
        ifstr >> point.intensity;
        
        if (local_normal.dot(local) > 0.0)
          local_normal = -local_normal;
        
        // This normal should point to the origin.
        Vector4d global_normal = transformation * local_normal;
        
        for (int i = 0; i < 3; ++i)
          point.normal(i) = global_normal(i);


        // Filter.
        //if (point.intensity < 40)
        //continue;
        points.push_back(point);
      }
    }
    ifstr.close();

    NormalizeIntensity(&points);

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
  
void InitializeFrame(const bool recompute, const string directory, const std::vector<Sweep>& sweeps,
                     const float average_distance, Frame* frame) {
  char buffer[1024];
  sprintf(buffer, "%s/frame.dat", directory.c_str());
  ifstream ifstr;
  ifstr.open(buffer);
  if (!recompute && ifstr.is_open()) {
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

  void InitializeEvidence(const bool recompute,
                          const std::vector<Sweep>& sweeps,
                        const Frame& frame,
                        const string directory,
                        vector<float>* point_evidence,
                        vector<float>* free_space_evidence,
                        vector<Vector3d>* normal_evidence) {
  const string point_evidence_filename = directory + "point_evidence.dat";
  const string free_space_evidence_filename = directory + "free_space_evidence.dat";
  const string normal_evidence_filename = directory + "normal_evidence.dat";
  ifstream ifstr0, ifstr1, ifstr2;
  ifstr0.open(point_evidence_filename.c_str());
  ifstr1.open(free_space_evidence_filename.c_str());
  ifstr2.open(normal_evidence_filename.c_str());

  if (!recompute && ifstr0.is_open() && ifstr1.is_open() && ifstr2.is_open()) {
    ifstr0.close();
    ifstr1.close();
    ifstr2.close();
    cerr << "Loading evidence..." << flush;
    LoadEvidence(point_evidence_filename, point_evidence);
    LoadEvidence(free_space_evidence_filename, free_space_evidence);
    LoadEvidence3(normal_evidence_filename, normal_evidence);

    DrawEvidenceToImage(frame, directory, *point_evidence, *free_space_evidence);
    
    cerr << "done." << endl;
  } else {
    ifstr0.close();
    ifstr1.close();
    cerr << "Computing evidence..." << flush;
    SetPointEvidence(sweeps, frame, directory, point_evidence, normal_evidence);
    SetFreeSpaceEvidence(sweeps, frame, directory, free_space_evidence);

    DrawEvidenceToImage(frame, directory, *point_evidence, *free_space_evidence);
    
    cerr << "done." << endl;
    WriteEvidence(point_evidence_filename, *point_evidence);
    WriteEvidence(free_space_evidence_filename, *free_space_evidence);
    WriteEvidence3(normal_evidence_filename, *normal_evidence);
  }
}

void RotateSweeps(const double angle, vector<Sweep>* sweeps) {
  Eigen::Matrix3f rotation;
  rotation <<
    cos(angle), -sin(angle), 0,
    sin(angle), cos(angle), 0,
    0, 0, 1;

  for (auto& sweep : *sweeps) {
    sweep.center = rotation * sweep.center;
    for (auto& point : sweep.points) {
      point.position = rotation * point.position;
      point.normal = rotation * point.normal;
    }
  }
}
  
}  // namespace

//----------------------------------------------------------------------
int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0]
         << " directory_prefix [recompute] [angle_in_degrees]" << endl
         << "./room_segmentation ../data/mobile/" << endl;
    return 1;
  }

  bool recompute = false;
  if (argc >= 3 && atoi(argv[2]))
    recompute = true;
  
  vector<Sweep> sweeps;
  ReadSweeps(argv[1], &sweeps);
  double angle = 0.0;
  if (argc >= 4)
    angle = atof(argv[3]) * M_PI / 180.0;
  RotateSweeps(angle, &sweeps);

  const float average_distance = ComputeAverageDistance(sweeps);
  cout << average_distance << endl;
  
  Frame frame;
  InitializeFrame(recompute, argv[1], sweeps, average_distance, &frame);

  ConvertSweepsToFrame(frame, &sweeps);
  
  vector<float> point_evidence, free_space_evidence;
  vector<Eigen::Vector3d> normal_evidence;
  InitializeEvidence(recompute, sweeps, frame, argv[1], &point_evidence, &free_space_evidence, &normal_evidence);

  vector<float> door_detection;
  DetectDoors(sweeps, frame, argv[1], point_evidence, free_space_evidence, &door_detection);
  
  return 0;
}
