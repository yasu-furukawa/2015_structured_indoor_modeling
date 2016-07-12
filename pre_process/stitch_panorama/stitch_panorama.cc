#include <fstream>
#include <list>
#include <opencv2/highgui/highgui.hpp>
#include "stitch_panorama.h"

using cv::imread;
using cv::imshow;
using cv::Mat;
using cv::waitKey;

using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using std::cerr;
using std::endl;
using std::flush;
using std::ifstream;
using std::list;
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

Eigen::Matrix3d RotationZ(const double radian) {
  Matrix3d rotation;
  rotation <<
    cos(radian), -sin(radian), 0,
    sin(radian), cos(radian), 0,
    0, 0, 1;

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
          ifstr >> rotations[c](x, y);
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

        if (0 <= pixel[0] && pixel[0] <= images[c].cols - 1 &&
            0 <= pixel[1] && pixel[1] <= images[c].rows - 1)
          masks[c].at<unsigned char>(y, x) = 255;
      }
    }

    // Set alpha.
    cv::Mat distance;
    distance.create(out_height, out_width, CV_32S);
    for (int y = 0; y < out_height; ++y) {
      for (int x = 0; x < out_width; ++x) {
        if (masks[c].at<unsigned char>(y, x) == 255)
          distance.at<short>(y, x) = -1;
        else
          distance.at<short>(y, x) = -2;
      }
    }

    list<Vector2i> front;
    for (int y = 1; y < out_height - 1; ++y) {
      for (int x = 0; x < out_width; ++x) {
        const int px = (x - 1 + out_width) % out_width;
        const int nx = (x + 1) % out_width;
        if (masks[c].at<unsigned char>(y, x) == 255 &&
            (masks[c].at<unsigned char>(y, px) == 0 ||
             masks[c].at<unsigned char>(y, nx) == 0 ||
             masks[c].at<unsigned char>(y - 1, x) == 0 ||
             masks[c].at<unsigned char>(y + 1, x) == 0)) {
          distance.at<short>(y, x) = 0;
          front.push_back(Vector2i(x, y));
        }
      }
    }
    while (!front.empty()) {
      const Vector2i pixel = front.front();
      front.pop_front();
      const int x = pixel[0];
      const int y = pixel[1];
      const int current_distance = distance.at<short>(y, x);
      if (current_distance >= margin)
        continue;

      for (int j = -1; j <= 1; ++j) {
        const int new_y = y + j;
        if (new_y < 0 || out_height <= new_y)
          continue;
        for (int i = -1; i <= 1; ++i) {
          const int new_x = (x + i + out_width) % out_width;
          if (distance.at<short>(new_y, new_x) == -1) {
            distance.at<short>(new_y, new_x) = current_distance + 1;
            front.push_back(Vector2i(new_x, new_y));
          }
        }
      }
      /*
      const int px = (x - 1 + out_width) % out_width;
      const int nx = (x + 1) % out_width;
      const int py = max(0, y - 1);
      const int ny = min(out_height - 1, y + 1);
      front.pop_front();

      const int current_distance = distance.at<short>(y, x);
      if (current_distance >= margin)
        continue;
      
      {
        if (distance.at<short>(y, px) == -1) {
          distance.at<short>(y, px) = current_distance + 1;
          front.push_back(Vector2i(px, y));
        }
      }
      {
        if (distance.at<short>(y, nx) == -1) {
          distance.at<short>(y, nx) = current_distance + 1;
          front.push_back(Vector2i(nx, y));
        }
      }
      {
        if (distance.at<short>(py, x) == -1) {
          distance.at<short>(py, x) = current_distance + 1;
          front.push_back(Vector2i(x, py));
        }
      }
      {
        if (distance.at<short>(ny, x) == -1) {
          distance.at<short>(ny, x) = current_distance + 1;
          front.push_back(Vector2i(x, ny));
        }
      }
      */
    }
    for (int y = 0; y < out_height; ++y) {
      for (int x = 0; x < out_width; ++x) {
        if (distance.at<short>(y, x) == -2)
          masks[c].at<unsigned char>(y, x) = 0;
        else if (distance.at<short>(y, x) == -1)
          masks[c].at<unsigned char>(y, x) = 255;
        else
          masks[c].at<unsigned char>(y, x) = distance.at<short>(y, x) * 255 / margin;
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

  const Matrix3d rotation = RotationZ(longitude) * RotationX(latitude);
  return rotation * Vector3d(0, 1, 0);
}
  
Eigen::Vector2d StitchPanorama::RayToScreen(const Eigen::Vector3d& ray) const {
  // cos -sin 0     1 0 0             0
  // sin cos  0     0 cosa -sina      1
  // 0   0    1     0 sina cosa       0
  //
  // y (-sin cosa, cos cosa, sina)
  const double sina = ray[2];
  const double cosa = sqrt(ray[0] * ray[0] + ray[1] * ray[1]);

  const double latitude = atan2(sina, cosa);
  double longitude;
  if (cosa == 0.0)
    longitude = 0.0;
  longitude = atan2(-ray[0] / cosa, ray[1] / cosa);
  if (longitude < 0.0)
    longitude += 2 * M_PI;

  const double x = min((double)out_width, longitude / (2 * M_PI) * out_width);
  const double y = max(0.0, min((double)out_height, (latitude + M_PI / 2.0) / M_PI * out_height));

  return Vector2d(x, y);  

  /*
  // cos 0 sin      1 0 0             0
  // 0 1 0        0 cosa -sina        0
  // -sin 0 cos    0 sina cosa        1
  //
  // z (sin cosa, -sina, cos cosa)
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
  */
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
  directory  = input.directory;
  out_width  = input.out_width;
  out_height = input.out_height;
  margin     = input.margin;
  if (!Init())
    return false;

  /*
  Vector3d axis0(0.93162, -0.35736, -0.06617);
  Vector3d axis1(0.9315, -0.35675  , -0.071);
  Vector3d axis2(0.92628, -0.36947, -0.07413);
  Vector3d axis3(0.91541, -0.39554, -0.07462);
  Vector3d axis4(0.89898, -0.43169, -0.07407);
  Vector3d axis5(0.8745, -0.47818, -0.08122);
  Vector3d axis6(0.84627 , -0.5267, -0.08004);
  Vector3d axis7(0.81547, -0.57273, -0.08358);
  Vector3d axis8(0.7911, -0.60633, -0.08077);
  Vector3d axis9(0.7545, -0.65019, -0.08941);
  Vector3d axis10(0.71475 , -0.6939, -0.08738);
  Vector3d axis11(0.67475, -0.73297, -0.08641);
  Vector3d axis12(0.63439, -0.76817, -0.08639);

  cerr << RayToScreen(axis0) << endl;
  cerr << RayToScreen(axis1) << endl;
  cerr << RayToScreen(axis2) << endl;
  cerr << RayToScreen(axis3) << endl;
  cerr << RayToScreen(axis4) << endl;
  cerr << RayToScreen(axis5) << endl;
  cerr << RayToScreen(axis6) << endl;
  cerr << RayToScreen(axis7) << endl;
  cerr << RayToScreen(axis8) << endl;
  cerr << RayToScreen(axis9) << endl;
  cerr << RayToScreen(axis10) << endl;
  cerr << RayToScreen(axis11) << endl;
  cerr << RayToScreen(axis12) << endl;
  //  return true;
  */
  /*
  for (int y = 10; y < out_height - 10; ++y) {
    for (int x = 0; x < out_width; ++x) {
      const auto ray = ScreenToRay(Vector2d(x, y));
      const auto pixel = RayToScreen(ray);
      cerr << (pixel - Vector2d(x, y)).norm() << ' ' << flush;
    }
  }
*/

  if (!SetMasks())
    return false;

  

  return false;
}

}  // namespace pre_process
