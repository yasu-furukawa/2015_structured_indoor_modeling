#include <iostream>
#include "navigation.h"

using namespace Eigen;
using namespace std;

namespace {

Eigen::Vector3d ComputeAirDirectionFromPanorama(const Eigen::Vector3d& panorama_direction,
                                                const double air_height,
                                                const double air_angle) {
  Vector3d direction = panorama_direction.normalized();
  // Lift up by the angle.
  direction += -tan(air_angle)* Vector3d(0, 0, 1);
  direction *= air_height / tan(air_angle);
  return direction;
}

Eigen::Vector3d ComputePanoramaDirectionFromAir(const Eigen::Vector3d& air_direction,
                                                const double distance) {
  Vector3d direction = air_direction;
  const int kZIndex = 2;
  direction[kZIndex] = 0.0;
  direction.normalize();
  direction *= distance;
  return direction;
}
  
}  // namespace

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
  case kAir: {
    return camera_air.GetCenter();
  }
  case kAirTransition: {
    const Vector3d direction = GetDirection();
    return camera_air.ground_center - direction;
  }
  case kPanoramaToAir: {
    const double weight_start =
      (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_between_panorama_and_air.camera_panorama.start_center +
      weight_end * (camera_between_panorama_and_air.camera_air.GetCenter());
  }
  case kAirToPanorama: {
    const double weight_start =
      (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_between_panorama_and_air.camera_air.GetCenter()) +
      weight_end * camera_between_panorama_and_air.camera_panorama.start_center;
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
    Vector3d horizontal_direction = camera_air.start_direction;
    const double vertical_distance = horizontal_direction[2];
    horizontal_direction[2] = 0.0;
    const double horizontal_distance = horizontal_direction.norm();
    
    const double weight_start = (cos(camera_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    Vector3d direction = weight_start * camera_air.start_direction +
      weight_end * camera_air.end_direction;
    direction[2] = 0.0;
    direction.normalize();
    direction *= horizontal_distance;
    direction[2] = vertical_distance;
    return direction;
    //return direction.normalized() *
    //((camera_air.start_direction.norm() + camera_air.end_direction.norm()) / 2.0);
  }
  case kPanoramaToAir: {
    const double weight_start =
      (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_between_panorama_and_air.camera_panorama.start_direction +
      weight_end * camera_between_panorama_and_air.camera_air.start_direction;
  }
  case kAirToPanorama: {
    const double weight_start =
      (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_between_panorama_and_air.camera_air.start_direction +
      weight_end * camera_between_panorama_and_air.camera_panorama.start_direction;
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

const CameraPanorama& Navigation::GetCameraPanorama() const {
  return camera_panorama;
}

const CameraAir& Navigation::GetCameraAir() const {
  return camera_air;
}

const CameraBetweenPanoramaAndAir& Navigation::GetCameraBetweenPanoramaAndAir() const {
  return camera_between_panorama_and_air;
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

  //----------------------------------------------------------------------
  air_height = 0.0;
  for (const auto& panorama_renderer : panorama_renderers) {
    air_height += panorama_renderer.GetAverageDistance();
  }
  air_height /= panorama_renderers.size();
  const double kHeightScale = 24.0;
  air_height *= kHeightScale;
  
  // air_angle = 45.0 * M_PI / 180.0;
  air_angle = 45.0 * M_PI / 180.0;

  average_distance = 0.0;
  for (const auto& panorama : panorama_renderers) {
    average_distance += panorama.GetAverageDistance();
  }
  average_distance /= static_cast<int>(panorama_renderers.size());
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
    const double kStepSize = 0.02;
    camera_between_panorama_and_air.progress += kStepSize;
    if (camera_between_panorama_and_air.progress >= 1.0) {
      camera_status = kAir;

      camera_air = camera_between_panorama_and_air.camera_air;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kAirToPanorama: {
    const double kStepSize = 0.02;
    camera_between_panorama_and_air.progress += kStepSize;
    if (camera_between_panorama_and_air.progress >= 1.0) {
      camera_status = kPanorama;

      camera_panorama = camera_between_panorama_and_air.camera_panorama;
      camera_panorama.progress = 0.0;
    }
    break;
  }
  default: {
    break;
  }
  }
}

void Navigation::RotatePanorama(const Eigen::Vector3d& axis)  {
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

void Navigation::MoveAir(const Eigen::Vector3d& translation)  {
  camera_air.ground_center += translation;
}

bool Navigation::Collide(const int from_index, const int to_index) const {
  return false;
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

void Navigation::MovePanorama(const Vector3d& direction) {
  const double kMaximumAngle = 60.0 * M_PI / 180.0;
  const double kPerpScale = 4.0;
  const int kInvalid = -1;
  int best_panorama_index = kInvalid;
  double best_distance = 0.0;

  Vector3d along_direction = direction;
  along_direction[2] = 0.0;
  along_direction.normalize();
  Vector3d perp_direction(along_direction[1], -along_direction[0], 0.0);

  for (int p = 0; p < static_cast<int>(panorama_renderers.size()); ++p) {
    if (p == camera_panorama.start_index)
      continue;
    // Behind.
    const Vector3d diff = panorama_renderers[p].GetCenter() - camera_panorama.start_center;
    
    if (diff.normalized().dot(direction.normalized()) <= cos(kMaximumAngle))
      continue;

    // Collision check.
    //????
    if (Collide(camera_panorama.start_index, p))
      continue;

    const double distance = diff.dot(along_direction) + kPerpScale * fabs(diff.dot(perp_direction));
    if (best_panorama_index == kInvalid || distance < best_distance) {
      best_panorama_index = p;
      best_distance = distance;
    }
  }

  if (best_panorama_index == kInvalid)
    return;

  MoveToPanorama(best_panorama_index);  
  // const int target_panorama_index = (camera_panorama.start_index + 1) % panorama_renderers.size();
  // MoveToPanorama(target_panorama_index);
}

void Navigation::MoveForwardPanorama() {
  MovePanorama(camera_panorama.start_direction);
}

void Navigation::MoveBackwardPanorama() {
  MovePanorama(-camera_panorama.start_direction);
}

void Navigation::RotatePanorama(const double radian) {
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

void Navigation::RotateSky(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_air.end_direction = rotation * camera_air.start_direction;
  camera_air.progress = 0.0;
  camera_status = kAirTransition;  
}

void Navigation::PanoramaToAir() {
  camera_status = kPanoramaToAir;

  camera_between_panorama_and_air.camera_panorama = camera_panorama;
  {
    CameraAir& camera_air      = camera_between_panorama_and_air.camera_air;
    camera_air.ground_center   =
      camera_panorama.start_center + camera_panorama.start_direction;
    camera_air.start_direction =
      ComputeAirDirectionFromPanorama(camera_panorama.start_direction,
                                      air_height,
                                      air_angle);

  }
  
  camera_between_panorama_and_air.progress = 0.0;  
}

void Navigation::AirToPanorama(const int panorama_index) {
  camera_status = kAirToPanorama;
  
  camera_between_panorama_and_air.camera_air = camera_air;
  {
    CameraPanorama& camera_panorama =
      camera_between_panorama_and_air.camera_panorama;
    camera_panorama.start_index     = panorama_index;
    camera_panorama.start_center    = panorama_renderers[panorama_index].GetCenter();
    camera_panorama.start_direction =
      ComputePanoramaDirectionFromAir(camera_air.start_direction,
                                      panorama_renderers[panorama_index].GetAverageDistance());
  }
  camera_between_panorama_and_air.progress = 0.0;  
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

double Navigation::GetFieldOfViewInDegrees() const {
  const double kPanoramaFieldOfView = 100.0;
  const double kAirFieldOfView = 10.0;

  switch (camera_status) {
  case kPanorama:
  case kPanoramaTransition: {
    return kPanoramaFieldOfView;
  }
  // CameraAir handles the state.
  case kAir:
  case kAirTransition: {
    return kAirFieldOfView;
  }
  // CameraBetweenGroundAndAir handles the state.
  case kPanoramaToAir: {
    //    const double weight_start = pow(1.0 - sin(camera_between_panorama_and_air.progress * M_PI / 2.0), 1.5);
    double weight_start;
    if (camera_between_panorama_and_air.progress < 0.5) {
      weight_start = 1.0;
    } else {
      weight_start = pow(1.0 - sin(2.0 * (camera_between_panorama_and_air.progress - 0.5) * M_PI / 2.0), 1.0);
    }
    // (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return weight_start * kPanoramaFieldOfView + weight_end * kAirFieldOfView;
  }
  case kAirToPanorama: {
    const double weight_start = 1.0 - pow(1.0 - sin((1.0 - camera_between_panorama_and_air.progress) * M_PI / 2.0), 1.5);
    const double weight_end = 1.0 - weight_start;
    // (cos(camera_between_panorama_and_air.progress * M_PI) + 1.0) / 2.0;
    // const double weight_end = 1.0 - weight_start;
    return weight_start * kAirFieldOfView + weight_end * kPanoramaFieldOfView;
  }
  }
}
