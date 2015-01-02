#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>
#include "../base/file_io.h"
#include "transformation.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

struct DepthPoint {
  int x;
  int y;
  double X;
  double Y;
  double Z;
  int red;
  int green;
  int blue;
  double distance;
};


void ReadPly(const string filename,
             int* depth_width,
             int* depth_height,
             vector<DepthPoint>* depth_points) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    cerr << "ply file does not exist: " << filename << endl;
    exit (1);
  }

  string stmp;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;
  int num_vertex;
  ifstr >> num_vertex;
  for (int i = 0; i < 37; ++i)
    ifstr >> stmp;
    
  const int kXOffset = 1;
  const int kYOffset = 1;
  
  int max_x = 0, max_y = 0;

  depth_points->resize(num_vertex);
  for (int i = 0; i < num_vertex; ++i) {
    ifstr >> depth_points->at(i).y >> depth_points->at(i).x
          >> depth_points->at(i).X >> depth_points->at(i).Y >> depth_points->at(i).Z
          >> depth_points->at(i).red >> depth_points->at(i).green >> depth_points->at(i).blue;
    
    depth_points->at(i).x -= kXOffset;
    depth_points->at(i).y -= kYOffset;
    double dummy;
    for (int j = 0; j < 4; ++j)
      ifstr >> dummy;
    
    const double distance = sqrt(depth_points->at(i).X * depth_points->at(i).X +
                                 depth_points->at(i).Y * depth_points->at(i).Y +
                                 depth_points->at(i).Z * depth_points->at(i).Z);
    depth_points->at(i).distance = distance;
      
    if (i == 0) {
      max_x = depth_points->at(i).x;
      max_y = depth_points->at(i).y;
    } else {
      max_x = max(depth_points->at(i).x, max_x);
      max_y = max(depth_points->at(i).y, max_y);
    }
  }
  ifstr.close();

  *depth_width = max_x + 1;
  *depth_height = max_y + 1;
}

Vector2d ProjectLocalToColor(const Vector3d& local_point,
                             const vector<double>& params,
                             const int color_width,
                             const int color_height) {
  const double color_phi_per_pixel = params[0] / color_height;
  const Matrix3d rx = RotationX(params[1]);
  const Matrix3d rz = RotationZ(params[2]);
  const Matrix3d ry = RotationY(params[3]);
  const Vector3d t(params[6], params[5], params[4]);
  const Matrix3d r = ry * rz * rx;

  Vector3d color_point = r * local_point + t;

  Vector2d color_uv;
  ConvertLocalToPanorama(color_width, color_height, color_phi_per_pixel, color_point, &color_uv);
  return color_uv;
}

int main(int argc, char* argv[]) {
  if (argc < 4) {
    cerr << "Usage: " << argv[0] << " data_directory from_pano to_pano" << endl;
    exit (1);
  }
  const int from_pano = atoi(argv[2]);
  const int to_pano = atoi(argv[3]);

  const FileIO file_io(argv[1]);

  vector<DepthPoint> depth_points;
  {
    int depth_width, depth_height;
    ReadPly(file_io.GetLocalPly(from_pano), &depth_width, &depth_height, &depth_points);
  }
  
  Matrix4d from_local_to_global;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetLocalToGlobalTransformation(from_pano));
    char ctmp;
    ifstr >> ctmp;
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 4; ++x) {
        ifstr >> from_local_to_global(y, x);
      }
    }
    ifstr.close();
    from_local_to_global(3, 0) = 0;
    from_local_to_global(3, 1) = 0;
    from_local_to_global(3, 2) = 0;
    from_local_to_global(3, 3) = 1;
  }

  Matrix4d to_local_to_global;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetLocalToGlobalTransformation(to_pano));
    char ctmp;
    ifstr >> ctmp;
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 4; ++x) {
        ifstr >> to_local_to_global(y, x);
      }
    }
    ifstr.close();
    to_local_to_global(3, 0) = 0;
    to_local_to_global(3, 1) = 0;
    to_local_to_global(3, 2) = 0;
    to_local_to_global(3, 3) = 1;
  }
  
  vector<double> params;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetPanoramaDepthAlignmentCalibration(to_pano));
    if (ifstr.is_open()) {
      string stmp;
      ifstr >> stmp;
      const int kNumParams = 7;
      params.resize(kNumParams);
      for (int i = 0; i < kNumParams; ++i)
        ifstr >> params[i];
      ifstr.close();
    }
  }

  // Create a canvas.
  const int kWidth = 1000;//3000;
  const int kHeight = 500; // 1500;
  const int kNumChannels = 3;
  vector<unsigned char> canvas(kWidth * kHeight * kNumChannels, 0);

  cerr << "Render " << depth_points.size() << " points." << endl;
  // For each point.
  int count = 0;
  for (const auto& point : depth_points) {
    if (count++ % 100 == 0)
      cerr << count << ' ' << flush;

    Vector4d from_local(point.X, point.Y, point.Z, 1.0);
    Vector4d global = from_local_to_global * from_local;

    Vector4d to_local = to_local_to_global.inverse() * global;  // ???
    Vector2d uv = ProjectLocalToColor(Vector3d(to_local[0],
                                               to_local[1],
                                               to_local[2]),
                                      params,
                                      kWidth,
                                      kHeight);
    const int u = static_cast<int>(round(uv[0]));
    const int v = max(0, min(kHeight - 1, static_cast<int>(round(uv[1]))));

    canvas[3 * (v * kWidth + u) + 0] = point.red;
    canvas[3 * (v * kWidth + u) + 1] = point.green;
    canvas[3 * (v * kWidth + u) + 2] = point.blue;
  }

  ofstream ofstr;
  ofstr.open("test.ppm");
  ofstr << "P3" << endl
        << kWidth << ' ' << kHeight << endl
        << 255 << endl;
  for (const auto intensity : canvas) {
    ofstr << static_cast<int>(intensity) << ' ';
  }
  ofstr.close();
}
