#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "stitch_panorama.h"

using cv::imread;
using cv::imshow;
using cv::Mat;
using cv::waitKey;

using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using std::cerr;
using std::endl;
using std::ifstream;
using std::max;
using std::min;
using std::string;
using std::vector;

namespace {

Eigen::Matrix3d RotationX(const double radian) {
  Matrix3d rotation;
  rotation <<
    1, 0, 0,
    0, cos(radian), -sin(radian),
    0, sin(radian), cos(radian);

  return rotation;
}

Eigen::Matrix3d RotationY(const double radian) {
  Matrix3d rotation;
  rotation <<
    cos(radian), 0, sin(radian),
    0, 1, 0,
    -sin(radian), 0, cos(radian);

  return rotation;
}

}  // namespace

namespace pre_process {

bool StitchPanorama::Init() {
  {
    ifstream ifstr;
    ifstr.open((directory + "/K.txt").c_str());
    if (!ifstr.is_open()) {
      cerr << "No K file." << endl;
      return false;
    }
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        ifstr >> intrinsics(y, x);
      }
    }
    ifstr.close();
  }

  // cerr << intrinsics << endl;

  {
    ifstream ifstr;
    ifstr.open((directory + "/IMU_rotation.txt").c_str());
    if (!ifstr.is_open()) {
      cerr << "No IMU_rotation file." << endl;
      return false;
    }
    ifstr >> num_cameras;
    rotations.resize(num_cameras);
    for (int c = 0; c < num_cameras; ++c) {
      for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
          ifstr >> rotations[c](y, x);
        }
      }
    }
    ifstr.close();
  }

  projections.resize(num_cameras);
  for (int c = 0; c < num_cameras; ++c) {
    projections[c] = intrinsics * rotations[c];
  }  

  images.resize(num_cameras);
  for (int c = 0; c < num_cameras; ++c) {
    char buffer[1024];
    sprintf(buffer, "%s/images/%04d.jpg", directory.c_str(), c);
    images[c] = imread(buffer, CV_LOAD_IMAGE_COLOR);
    // imshow("Input Image", images[c]);
    // waitKey(0);
  }  

  return true;
}
  
bool StitchPanorama::SetMasks() {
  masks.resize(num_cameras);
  for (int c = 0; c < num_cameras; ++c) {
    masks[c].create(out_height, out_width, CV_8UC(1));
    for (int y = 0; y < out_height; ++y) {
      for (int x = 0; x < out_width; ++x) {
        masks[c].at<unsigned char>(y, x) = 0;
      }
    }

    for (int y = 0; y < out_height; ++y) {
      for (int x = 0; x < out_width; ++x) {
        const Vector3d ray = ScreenToRay(Vector2d(x, y));
        const Vector3d pixel = Project(c, ray);
        if (pixel[2] <= 0.0)
          continue;

        
        
        
        if ((y / 10) % 2 == 0 && (x / 10) % 2 == 0)
          masks[c].at<unsigned char>(y, x) = 255;
        //else
        //masks[c].at<unsigned char>(y, x) = 0;
      }
    }
    imshow("A", masks[c]);
    waitKey(0);
  }

  return true;
}

Eigen::Vector3d StitchPanorama::ScreenToRay(const Eigen::Vector2d& screen) const {
  const double longitude = screen[0] * 2.0 * M_PI / out_width;
  const double latitude  = screen[1] * M_PI / out_height - (M_PI / 2.0);

  const Matrix3d rotation = RotationY(longitude) * RotationX(latitude);
  return rotation * Vector3d(0, 0, 1);
}
  
Eigen::Vector2d StitchPanorama::RayToScreen(const Eigen::Vector3d& ray) const {
  // cos 0 sin      1 0 0               0
  // 0 1 0        0 cosa -sina        0
  // -sin 0 cos    0 sina cosa         1
  //
  // (sin cosa, -sina, cos cosa)

  const double sina = -ray[1];
  const double cosa = sqrt(ray[0] * ray[0] + ray[2] * ray[2]);
  const double latitude = atan2(sina, cosa);
  double longitude;
  if (cosa == 0.0)
    longitude = 0.0;
  longitude = atan2(ray[0] / cosa, ray[2] / cosa);
  if (longitude < 0.0)
    longitude += 2 * M_PI;

  const double x = min((double)out_width, longitude / (2 * M_PI) * out_width);
  const double y = max(0.0, min((double)out_height, (latitude + M_PI / 2.0) / M_PI * out_height));

  return Vector2d(x, y);  
}

Eigen::Vector3d StitchPanorama::Project(const int camera, const Eigen::Vector3d& ray) const {
  Vector3d pixel = projections[camera] * ray;
  if (pixel[2] != 0.0) {
    pixel[0] /= pixel[2];
    pixel[1] /= pixel[2];
  }
  return pixel;
}
  
bool StitchPanorama::Stitch(const Input& input) {
  directory = input.directory;
  out_width = input.out_width;
  out_height = input.out_height;
  if (!Init())
    return false;

  if (!SetMasks())
    return false;

  

  return false;
}

}  // namespace pre_process
