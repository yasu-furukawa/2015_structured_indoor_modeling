#ifndef NAVIGATION_H__
#define NAVIGATION_H__

#include <Eigen/Dense>
#include <vector>

#include "panorama_renderer.h"

enum CameraStatus {
  kPanorama,
  kPanoramaTransition,
  kAir,
  kAirTransition,
  kPanoramaToAir,
  kAirToPanorama
};

struct CameraOnGround {
  int start_index;
  Eigen::Vector3d start_center;
  Eigen::Vector3d start_direction;
  
  int end_index;
  Eigen::Vector3d end_center;
  Eigen::Vector3d end_direction;
  
  double progress;
};

struct CameraOnAir {
  Eigen::Vector3d ground_center;
  Eigen::Vector3d start_direction;
  Eigen::Vector3d end_direction;

  double progress;
};

class Navigation {
 public:
  Navigation(const std::vector<PanoramaRenderer>& panorama_renderers);

  // Get the current camera center.
  Eigen::Vector3d GetCenter() const;
  Eigen::Vector3d GetDirection() const;
  CameraStatus GetCameraStatus() const;
  CameraOnGround GetCameraOnGround() const;
  CameraOnAir GetCameraOnAir() const;

  void Init();

  void Tick();
  
  void RotateOnGround(const Eigen::Vector3d& axis);
  void MoveForwardOnGround();
  void MoveBackwardOnGround();
  void RotateOnGround(const double radian);

  void PanoramaToAir();
  void AirToPanorama();

  double Progress() const;
  
 private:
  void MoveToPanorama(const int target_panorama_index);
  void AllocateResources();
  void FreeResources();
  void SetMatrices();
  
  // Camera is at (center) and looks along (direction).
  CameraStatus camera_status;
  CameraOnGround camera_on_ground;
  CameraOnAir camera_on_air;
  int current_width;
  int current_height;

  const std::vector<PanoramaRenderer>& panorama_renderers;
};

#endif  // NAVIGATION_H__
