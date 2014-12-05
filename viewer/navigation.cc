#include <iostream>
#include "navigation.h"

using namespace Eigen;
using namespace std;

Navigation::Navigation(const vector<PanoramaRenderer>& panorama_renderers)
  : panorama_renderers(panorama_renderers) {
}

Vector3d Navigation::GetCenter() const {
  switch (camera_status) {
  case kPanorama: {
    return camera_panorama.start_center;
  }
  case kPanoramaTransition: {
    const double weight_start = (cos(camera_panorama.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return weight_start * camera_panorama.start_center +
      weight_end * camera_panorama.end_center;
  }
  case kAir:
  case kAirTransition: {
    return camera_air.ground_center - camera_air.start_direction;
  }
  default: {
    cerr << "Invalid camera_status." << endl;
    exit (1);
  }
  }
}

Vector3d Navigation::GetDirection() const {
  switch (camera_status) {
  case kPanorama: {
    return camera_panorama.start_direction;
  }
  case kPanoramaTransition: {
    // Interpolation.
    const double weight_start = (cos(camera_panorama.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    const Vector3d direction = weight_start * camera_panorama.start_direction +
      weight_end * camera_panorama.end_direction;
    return direction.normalized();
  }
  case kAir: {
    return camera_air.start_direction;
  }
  case kAirTransition: {
    const double weight_start = (cos(camera_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    const Vector3d direction = weight_start * camera_air.start_direction +
      weight_end * camera_air.end_direction;
    return direction.normalized() * ((camera_air.start_direction.norm() + camera_air.end_direction.norm()) / 2.0);
  }
  default: {
    cerr << "Invalid camera_status." << endl;
    exit (1);
  }
  }
}

CameraStatus Navigation::GetCameraStatus() const {
  return camera_status;
}

CameraPanorama Navigation::GetCameraPanorama() const {
  return camera_panorama;
}

CameraAir Navigation::GetCameraAir() const {
  return camera_air;
}

void Navigation::Init() {
  if (panorama_renderers.empty()) {
    cerr << "No panoramas." << endl;
    exit (1);
  }

  const int kStartIndex = 0;
  camera_status = kPanorama;
  camera_panorama.start_index = kStartIndex;
  camera_panorama.start_center = panorama_renderers[kStartIndex].GetCenter();
  camera_panorama.start_direction =
    panorama_renderers[kStartIndex].GetAverageDistance() * Vector3d(1, 0, 0);
  camera_panorama.progress = 0.0;
}

void Navigation::Tick() {
  switch (camera_status) {
  case kPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_panorama.progress += kStepSize;
    if (camera_panorama.progress >= 1.0) {
      camera_status = kPanorama;

      camera_panorama.start_index = camera_panorama.end_index;
      camera_panorama.start_center = camera_panorama.end_center;
      camera_panorama.start_direction = camera_panorama.end_direction;
      camera_panorama.progress = 0.0;
    }
    break;
  }
  case kAirTransition: {
    const double kStepSize = 0.02;
    camera_air.progress += kStepSize;
    if (camera_air.progress >= 1.0) {
      camera_status = kAir;

      camera_air.start_direction = camera_air.end_direction;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kPanoramaToAir: {
    /*
    const double kStepSize = 0.02;
    camera_air.progress += kStepSize;
    if (camera_air.progress >= 1.0) {
      camera_status = kAir;

      camera_air.start_direction = camera_air.end_direction;
      camera_air.progress = 0.0;
    }
    */
    break;
  }
  case kAirToPanorama: {
    //????
    break;
  }
  default: {
    break;
  }
  }
}

void Navigation::RotateOnGround(const Eigen::Vector3d& axis)  {
  Eigen::Matrix3d x_rotate;
  x_rotate << 
    cos(axis[0]), -sin(axis[0]), 0,
    sin(axis[0]), cos(axis[0]), 0,
    0, 0, 1;

  Eigen::Matrix3d y_rotate;
  y_rotate << cos(axis[1]), 0, sin(axis[1]),
    0, 1, 0,
    -sin(axis[1]), 0, cos(axis[1]);

  camera_panorama.start_direction = x_rotate * y_rotate * camera_panorama.start_direction;
}

void Navigation::MoveToPanorama(const int target_panorama_index) {
  cout << "move " << target_panorama_index << endl;
  camera_panorama.end_index = target_panorama_index;
  camera_panorama.end_center = panorama_renderers[target_panorama_index].GetCenter();

  Vector3d movement = panorama_renderers[target_panorama_index].GetCenter() -
    panorama_renderers[camera_panorama.start_index].GetCenter();
  movement.normalize();

  Vector3d sum;
  if (camera_panorama.start_direction.dot(movement) > 0.0)
    sum = camera_panorama.start_direction + movement;
  else
    sum = camera_panorama.start_direction - movement;
  // Makes it more horizontal.
  sum[2] /= 2.0;
  sum.normalize();
  camera_panorama.end_direction =
    sum * panorama_renderers[target_panorama_index].GetAverageDistance();

  // Starts animation.
  camera_panorama.progress = 0.0;
  camera_status = kPanoramaTransition;
}

void Navigation::MoveForwardOnGround() {
  const int target_panorama_index = (camera_panorama.start_index + 1) % panorama_renderers.size();
  MoveToPanorama(target_panorama_index);
}

void Navigation::MoveBackwardOnGround() {
  const int target_panorama_index =
    (camera_panorama.start_index - 1 + panorama_renderers.size()) % panorama_renderers.size();
  MoveToPanorama(target_panorama_index);
}

void Navigation::RotateOnGround(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_panorama.end_index = camera_panorama.start_index;
  camera_panorama.end_center = camera_panorama.start_center;
  camera_panorama.end_direction = rotation * camera_panorama.start_direction;
  camera_panorama.progress = 0.0;
  camera_status = kPanoramaTransition;  
}

void Navigation::PanoramaToAir() {
  camera_status = kPanoramaToAir;
  
}

void Navigation::AirToPanorama() {
  camera_status = kAirToPanorama;

}
 
double Navigation::Progress() const {
  switch (camera_status) {
  case kPanoramaTransition:
    return cos(camera_panorama.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kAirTransition:
    return cos(camera_air.progress * M_PI) / 2.0 + 1.0 / 2.0;
  default:
    cerr << "Impossible in Progress." << endl;
    exit (1);
    // return 0.0;
  }
}
