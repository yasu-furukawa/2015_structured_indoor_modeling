#include <Eigen/Dense>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include "data.h"

using namespace std;
using namespace floored;

namespace floored {

void ReadPly(ifstream* ifstr_ply, Sweep* sweep)  {
  ifstream& ifstr = *ifstr_ply;
  string header;
  int num_vertex;
  for (int i = 0; i < 6; ++i)
    ifstr >> header;
  ifstr >> num_vertex;

  int xyz_index[3];
  int nxnynz_index[3];
  int property_count = 0;
  for (property_count = 0; ; ++property_count) {
    ifstr >> header;
    if (header == "end_header")
      break;
    ifstr >> header >> header;
    if (header == "x")
      xyz_index[0] = property_count;
    else if (header == "y")
      xyz_index[1] = property_count;
    else if (header == "z")
      xyz_index[2] = property_count;
    else if (header == "nx")
      nxnynz_index[0] = property_count;
    else if (header == "ny")
      nxnynz_index[1] = property_count;
    else if (header == "nz")
      nxnynz_index[2] = property_count;
  }

  sweep->center = Eigen::Vector3f(0, 0, 0);
  sweep->points.resize(num_vertex);

  cerr << "Reading " << num_vertex << " points..." << endl;
  for (int i = 0;  i < num_vertex; ++i) {
    vector<float> properties(property_count);
    for (int j = 0; j < property_count; ++j) {
      ifstr >> properties[j];
    }
    for (int j = 0; j < 3; ++j) {
      sweep->points[i].position[j] = properties[xyz_index[j]];
      sweep->points[i].normal[j] = properties[nxnynz_index[j]];
    }
  }
}

void ReadTransform(ifstream* ifstr_transform, Eigen::Matrix4f* transform) {
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      (*ifstr_transform) >> (*transform)(y, x);
}

Eigen::Vector3f TransformPoint(const Eigen::Matrix4f& transform, Eigen::Vector3f& point) {
  Eigen::Vector4f point4(point[0], point[1], point[2], 1.0);
  point4 = transform * point4;
  return Eigen::Vector3f(point4[0], point4[1], point4[2]);
}

Eigen::Vector3f TransformNormal(const Eigen::Matrix4f& transform, Eigen::Vector3f& normal) {
  Eigen::Vector4f normal4(normal[0], normal[1], normal[2], 0.0);
  normal4 = transform * normal4;
  return Eigen::Vector3f(normal4[0], normal4[1], normal4[2]);
}

void ApplyTransform(const Eigen::Matrix4f& transform, Sweep* sweep) {
  sweep->center = TransformPoint(transform, sweep->center);
  for (int i = 0; i < sweep->points.size(); ++i) {
    sweep->points[i].position = TransformPoint(transform, sweep->points[i].position);
    sweep->points[i].normal   = TransformNormal(transform, sweep->points[i].normal);
  }
}

void WriteSweep(const Sweep& sweep, const string filename) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "element vertex " << sweep.points.size() + 1 << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property float nx" << endl
        << "property float ny" << endl
        << "property float nz" << endl
        << "end_header" << endl;

  ofstr << sweep.center << " 0 0 0" << endl;
  for (int p = 0; p < sweep.points.size(); ++p) {
    ofstr << sweep.points[p].position << ' '
          << sweep.points[p].normal << endl;
  }

  ofstr.close();
}

}  // namespace floored

int main(int argc, char* argv[]) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0]
         << " data_dir max_sweep_id" << endl;
    return 1;
  }

  const int max_sweep_id = atoi(argv[2]);

  ofstream ofstr;
  char buffer[1024];
  sprintf(buffer, "%s/ply/point.pset", argv[1]);
  ofstr.open(buffer);
  
  for (int s = 0; s < max_sweep_id; ++s) {
    char ply[1024];
    char transform[1024];
    sprintf(ply, "%s/ply/%03d.ply", argv[1], s);
    sprintf(transform, "%s/transformations/sweep%03d_transf.txt", argv[1], s);

    ifstream ifstr_ply, ifstr_transform;
    ifstr_ply.open(ply);
    ifstr_transform.open(transform);

    if (ifstr_ply.is_open() && ifstr_transform.is_open()) {
      Sweep sweep;
      ReadPly(&ifstr_ply, &sweep);
      Eigen::Matrix4f transform;
      ReadTransform(&ifstr_transform, &transform);
      ApplyTransform(transform, &sweep);

      char output_ply[1024];
      sprintf(output_ply, "%s/ply/transformed_%03d.ply", argv[1], s);
      WriteSweep(sweep, output_ply);

      for (int i = 0; i < sweep.points.size(); ++i) {
        ofstr << sweep.points[i].position[0] << ' '
              << sweep.points[i].position[1] << ' '
              << sweep.points[i].position[2] << ' '
              << sweep.points[i].normal[0] << ' '
              << sweep.points[i].normal[1] << ' '
              << sweep.points[i].normal[2] << endl;
      }
      
    } else if (ifstr_ply.is_open() || ifstr_transform.is_open()) {
      cerr << "Either ply or transform is missing: "
           << ply << ' ' << transform << endl;
      exit (1);
    }

    ifstr_ply.close();
    ifstr_transform.close();
  }

  ofstr.close();  

  return 0;
}
