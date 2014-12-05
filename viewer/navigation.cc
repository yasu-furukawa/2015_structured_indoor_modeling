#include <iostream>
#include "navigation.h"

using namespace Eigen;
using namespace std;

Navigation::Navigation(const vector<PanoramaRenderer>& panorama_renderers)
  : panorama_renderers(panorama_renderers) {
}

Vector3d Navigation::GetCenter() const {
  if (camera_status == kPanoramaStop)
    return camera_on_ground.start_center;
  else {
    // Interpolation.
    const double weight_start = (cos(camera_on_ground.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;

    return weight_start * camera_on_ground.start_center +
      weight_end * camera_on_ground.end_center;
  }
}

Vector3d Navigation::GetDirection() const {
  if (camera_status == kPanoramaStop)
    return camera_on_ground.start_direction;
  else {
    // Interpolation.
    const double weight_start = (cos(camera_on_ground.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;

    Vector3d direction = weight_start * camera_on_ground.start_direction +
      weight_end * camera_on_ground.end_direction;
    return direction.normalized();
  }
}

CameraStatus Navigation::GetCameraStatus() const {
  return camera_status;
}

CameraOnGround Navigation::GetCameraOnGround() const {
  return camera_on_ground;
}

void Navigation::Init() {
  if (panorama_renderers.empty()) {
    cerr << "No panoramas." << endl;
    exit (1);
  }

  const int kStartIndex = 0;
  camera_status = kPanoramaStop;
  camera_on_ground.start_index = kStartIndex;
  camera_on_ground.start_center = panorama_renderers[kStartIndex].GetCenter();
  camera_on_ground.start_direction = Vector3d(1, 0, 0);
  camera_on_ground.progress = 0.0;
}

void Navigation::Tick() {
  if (camera_status == kPanoramaTransition) {
    const double kStepSize = 0.02;
    camera_on_ground.progress += kStepSize;
    if (camera_on_ground.progress >= 1.0) {
      camera_status = kPanoramaStop;

      camera_on_ground.start_index = camera_on_ground.end_index;
      camera_on_ground.start_center = camera_on_ground.end_center;
      camera_on_ground.start_direction = camera_on_ground.end_direction;
      camera_on_ground.progress = 0.0;
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

  camera_on_ground.start_direction = x_rotate * y_rotate * camera_on_ground.start_direction;
}

void Navigation::MoveToPanorama(const int target_panorama_index) {
  cout << "move " << target_panorama_index << endl;
  camera_on_ground.end_index = target_panorama_index;
  camera_on_ground.end_center = panorama_renderers[target_panorama_index].GetCenter();

  Vector3d movement = panorama_renderers[target_panorama_index].GetCenter() -
    panorama_renderers[camera_on_ground.start_index].GetCenter();
  movement.normalize();

  Vector3d sum;
  if (camera_on_ground.start_direction.dot(movement) > 0.0)
    sum = camera_on_ground.start_direction + movement;
  else
    sum = camera_on_ground.start_direction - movement;
  // Makes it more horizontal.
  sum[2] /= 2.0;
  sum.normalize();
  camera_on_ground.end_direction = sum;

  // Starts animation.
  camera_on_ground.progress = 0.0;
  camera_status = kPanoramaTransition;
}

void Navigation::MoveForwardOnGround() {
  const int target_panorama_index = (camera_on_ground.start_index + 1) % panorama_renderers.size();
  MoveToPanorama(target_panorama_index);
}

void Navigation::MoveBackwardOnGround() {
  const int target_panorama_index =
    (camera_on_ground.start_index - 1 + panorama_renderers.size()) % panorama_renderers.size();
  MoveToPanorama(target_panorama_index);
}

void Navigation::RotateOnGround(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_on_ground.end_index = camera_on_ground.start_index;
  camera_on_ground.end_center = camera_on_ground.start_center;
  camera_on_ground.end_direction = rotation * camera_on_ground.start_direction;
  camera_on_ground.progress = 0.0;
  camera_status = kPanoramaTransition;  
}

double Navigation::Progress() const {
  return cos(camera_on_ground.progress * M_PI) / 2.0 + 1.0 / 2.0;
}
