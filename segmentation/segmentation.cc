#include <fstream>
#include <iostream>

#include "evidence.h"
#include "segmentation.h"

using namespace Eigen;
using namespace std;

namespace segmentation {

void ReadSweeps(const file_io::FileIO& file_io,
                vector<Sweep>* sweeps) {
  sweeps->clear();
  cerr << "Reading points... " << flush;

  const int kMaxNum = 200;
  for (int s = 0; s < kMaxNum; ++s) {    
    ifstream ifstr;
    ifstr.open(file_io.GetLocalPly(s).c_str());
    if (!ifstr.is_open())
      continue;
    cerr << "Reading... " << file_io.GetLocalPly(s) << endl;

    Sweep sweep;    
    {
      string header;
      for (int i = 0; i < 6; ++i)
        ifstr >> header;
      int num_points;
      ifstr >> num_points;
      for (int i = 0; i < 37; ++i)
        ifstr >> header;

      sweep.points.resize(num_points);
      for (int p = 0; p < num_points; ++p) {
        int itmp;
        ifstr >> itmp >> itmp;
        for (int i = 0; i < 3; ++i)
          ifstr >> sweep.points[p].position[i];
        
        double dtmp;
        for (int i = 0; i < 3; ++i)
          ifstr >> dtmp;
        
        for (int i = 0; i < 3; ++i)
          ifstr >> sweep.points[p].normal[i];

        ifstr >> sweep.points[p].weight;
        if (sweep.points[p].position.dot(sweep.points[p].normal) > 0.0)
          sweep.points[p].normal = -sweep.points[p].normal;
      }
    }
    ifstr.close();

    Matrix4d local_to_global;
    {
      ifstream ifstr;
      ifstr.open(file_io.GetLocalToGlobalTransformation(s));
      char ctmp;
      ifstr >> ctmp;
      for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 4; ++x)
          ifstr >> local_to_global(y, x);
      local_to_global(3, 0) = 0;
      local_to_global(3, 1) = 0;
      local_to_global(3, 2) = 0;
      local_to_global(3, 3) = 1;
      ifstr.close();
    }
    sweep.center = Vector3d(local_to_global(0, 3),
                            local_to_global(1, 3),
                            local_to_global(2, 3));
    
    ConvertSweepFromLocalToGlobal(local_to_global, &sweep);    
    sweeps->push_back(sweep);
  }
}

void NormalizeIntensity(const double min_sigma,
                        const double max_sigma,
                        Sweep* sweep) {
  double average_intensity = 0.0;
  int denom = 0;
  for (const auto& point : sweep->points) {
    if (point.weight != 0.0) {
      average_intensity += point.weight;
      ++denom;
    }
  }

  if (denom == 0.0) {
    cerr << "No non-zero intensity point?" << endl;
    exit (1);
  }
  average_intensity /= denom;
  double deviation = 0.0;
  for (const auto& point : sweep->points) {
    if (point.weight != 0.0) {
      deviation += (point.weight - average_intensity) *
        (point.weight - average_intensity);
    }
  }
  deviation /= denom;
  deviation = sqrt(deviation);

  for (auto& point : sweep->points) {
    point.weight =
      (point.weight - average_intensity) / deviation;
    point.weight =
      255 * max(0.0, min(1.0, (point.weight - min_sigma) / (max_sigma - min_sigma)));
  }

  //----------------------------------------------------------------------
  vector<OrientedPoint> points;
  points.swap(sweep->points);
  for (const auto& point : points) {
    if (point.weight != 0.0)
      sweep->points.push_back(point);
  }
}

void ConvertSweepFromLocalToGlobal(const Eigen::Matrix4d& local_to_global,
                                   Sweep* sweep) {
  sweep->center = ConvertPoint(sweep->center, local_to_global);
  for (auto& point : sweep->points) {
    point.position = ConvertPoint(point.position, local_to_global);
    point.normal   = ConvertNormal(point.normal, local_to_global);
  }
}

void RotateSweep(const double angle, Sweep* sweep) {
  Eigen::Matrix3d rotation;
  rotation <<
    cos(angle), -sin(angle), 0,
    sin(angle), cos(angle), 0,
    0, 0, 1;

  sweep->center = rotation * sweep->center;
  for (auto& point : sweep->points) {
    point.position = rotation * point.position;
    point.normal = rotation * point.normal;
  }
}

float ComputeAverageDistance(const std::vector<Sweep>& sweeps) {
  float average_distance = 0.0;
  int count = 0;

  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      average_distance += (point.position - sweep.center).norm();
      ++count;
    }
  }
  
  if (count == 0) {
    cerr << "No points?" << endl;
    return 1.0;
  }
  
  return average_distance / count;
}

void InitializeFrame(const bool recompute,
                     const string directory,
                     const std::vector<Sweep>& sweeps,
                     const float average_distance,
                     Frame* frame) {
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

void ComputeFrame(const std::string directory,
                  const vector<Sweep>& sweeps,
                  const float average_distance,
                  Frame* frame) {
  frame->axes[0] = Vector3d(1, 0, 0);
  frame->axes[1] = Vector3d(0, 1, 0);
  frame->axes[2] = Vector3d(0, 0, 1);

  // Set frame and apply ToLocal.
  SetRanges(sweeps, average_distance, frame);
}

void SetRanges(const vector<Sweep>& sweeps,
               const float average_distance,
               Frame* frame) {
  // Do not take into account point weights.
  vector<double> histogram[3];

  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      for (int a = 0; a < 3; ++a) {
        histogram[a].push_back(point.position.dot(frame->axes[a]));
      }
    }
  }

  for (int a = 0; a < 3; ++a) {
    sort(histogram[a].begin(), histogram[a].end());

    // Take the 5 and 95 percentiles.
    const auto pos_5  = histogram[a].begin() + (histogram[a].size() * 1 / 100);
    const auto pos_95 = histogram[a].begin() + (histogram[a].size() * 99 / 100);

    nth_element(histogram[a].begin(), pos_5,  histogram[a].end());
    nth_element(histogram[a].begin(), pos_95, histogram[a].end());

    const double diff = (*pos_95) - (*pos_5);
    // Allow a margin of 5 percents.
    const double margin = diff * 5 / 100;

    frame->ranges[a][0] = (*pos_5)  - margin;
    frame->ranges[a][1] = (*pos_95) + margin;
  }

  //----------------------------------------------------------------------
  // Initial guess of unit.
  //???????
  //double unit = average_distance / 150.0;
  double unit = average_distance / 100.0;
  //double unit = average_distance / 50.0;
  // Compute resolution.
  const int width  = static_cast<int>(round((frame->ranges[0][1] - frame->ranges[0][0]) / unit));
  const int height = static_cast<int>(round((frame->ranges[1][1] - frame->ranges[1][0]) / unit));
  // const int kMaxResolution = 600;  // autodesk
  const int kMaxResolution = 1024;
  // Don't use depth.
  const int max_current_resolution = max(width, height);
  if (kMaxResolution < max_current_resolution) {
    unit *= max_current_resolution / kMaxResolution;
  }

  frame->unit = unit;

  for (int a = 0; a < 3; ++a) {
    frame->size[a] = static_cast<int>(round((frame->ranges[a][1] - frame->ranges[a][0]) / unit));
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
  
  
}  // namespace segmentation
