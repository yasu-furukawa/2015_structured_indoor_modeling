#include "transform.h"

#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace floored {

void ReadTransforms(const string directory, const int num, const int reference_num,
                    vector<Matrix4f>* transforms) {
  transforms->clear();
  for (int s = 1; s <= num; ++s) {
    // Identify matrix if s is a reference.
    if (s == reference_num) {
      transforms->push_back(Matrix4f::Identity());
    } else {
      char buffer[1024];
      sprintf(buffer,
              "%s/transforms/sweep_%03d_sweep_%03d.txt",
              directory.c_str(), s, reference_num);
      ifstream ifstr;
      ifstr.open(buffer);
      if (!ifstr.is_open()) {
        cerr << "Transform file does not exist: " << buffer;
        exit (1);
      }

      Matrix4f transform;
      for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
          ifstr >> transform(y, x);;

      transforms->push_back(transform);      
      ifstr.close();
    }
  }
}
 
Vector3f TransformPoint(const Matrix4f& transform, const Vector3f& point) {
  Vector4f point4(point[0], point[1], point[2], 1.0f);
  point4 = transform * point4;
  return Vector3f(point4[0], point4[1], point4[2]);
}

void TransformSweeps(const vector<Matrix4f>& transforms,
                     vector<Sweep>* sweeps) {
  if (transforms.size() != sweeps->size()) {
    cerr << "Array dimensions do not agree: " << transforms.size() << ' ' << sweeps->size() << endl;
    exit (1);
  }

  for (int s = 0; s < transforms.size(); ++s) {
    for (auto& point : sweeps->at(s).points) {
      point.position = TransformPoint(transforms[s], point.position);
      point.normal   = TransformPoint(transforms[s], point.normal);
    }
    sweeps->at(s).center = TransformPoint(transforms[s], sweeps->at(s).center);
  }
}

void ConvertSweepsToFrame(const Frame& frame, vector<Sweep>* sweeps) {
  for (auto& sweep : (*sweeps)) {
    for (auto& point : sweep.points) {
      point.position = frame.ToLocalPosition(point.position);
      point.normal   = frame.ToLocalNormal(point.normal);
    }
    sweep.center = frame.ToLocalPosition(sweep.center);
  }
}
  
}  // namespace floored
