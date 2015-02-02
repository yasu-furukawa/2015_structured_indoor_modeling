#ifndef NAVIGATION_H__
#define NAVIGATION_H__

#include <Eigen/Dense>
#include <map>
#include <vector>

namespace structured_indoor_modeling {

class Floorplan;
class PanoramaRenderer;
class PolygonRenderer;

enum CameraStatus {
  // CameraPanorama handles the states.
  kPanorama,
  kPanoramaTransition,
  // CameraAir handles the state.
  kAir,
  kAirTransition,
  // CameraFloorplan handles the state.
  
  
  // CameraBetweenGroundAndAir handles the state.
  kPanoramaToAirTransition,
  kAirToPanoramaTransition,
  // Tour
  kPanoramaTour
};

struct CameraPanorama {
  int start_index;
  Eigen::Vector3d start_center;
  Eigen::Vector3d start_direction;
  
  int end_index;
  Eigen::Vector3d end_center;
  Eigen::Vector3d end_direction;
  
  double progress;
};

struct CameraAir {
  Eigen::Vector3d ground_center;
  Eigen::Vector3d start_direction;
  Eigen::Vector3d end_direction;

  double progress;

  Eigen::Vector3d GetCenter() const {
    return ground_center - start_direction;
  }
};

// struct CameraFloorplan {
// };

struct CameraBetweenPanoramaAndAir {
  // "progress" is not used in camera_panorama and camera_air.
  CameraPanorama camera_panorama;
  CameraAir camera_air;
  double progress;  
};

struct CameraPanoramaTour {
  std::vector<int> indexes;
  std::vector<Eigen::Vector3d> centers;
  std::vector<Eigen::Vector3d> directions;

  void GetIndexWeightPairs(const double progress,
                           int index_pair[2],
                           int panorama_index_pair[2],
                           double weight_pair[2]) const;
  Eigen::Vector3d GetCenter(const double progress) const;
  Eigen::Vector3d GetDirection(const double progress) const;
  
  double progress;
};

class Navigation {
 public:
  Navigation(const std::vector<PanoramaRenderer>& panorama_renderers,
             const PolygonRenderer& polygon_renderer,
             const std::map<int, int>& panorama_to_room,
             const std::map<int, int>& room_to_panorama);

  // Accessors.
  Eigen::Vector3d GetCenter() const;
  Eigen::Vector3d GetDirection() const;
  CameraStatus GetCameraStatus() const;
  const CameraPanorama& GetCameraPanorama() const;
  const CameraAir& GetCameraAir() const;
  const CameraBetweenPanoramaAndAir& GetCameraBetweenPanoramaAndAir() const;
  const CameraPanoramaTour& GetCameraPanoramaTour() const;

  // double Progress() const;
  double ProgressInverse() const;
  double GetFieldOfViewInDegrees() const;
  double GetAverageDistance() const { return average_distance; }

  //----------------------------------------------------------------------
  void Init();
  void Tick();

  // Interactions.
  void RotatePanorama(const double dx, const double dy);
  void MovePanorama(const Eigen::Vector3d& direction);
  void MoveToPanorama(const int target_panorama_index);
  void TourToPanorama(const std::vector<int>& indexes);
  void MoveForwardPanorama();
  void MoveBackwardPanorama();
  void RotatePanorama(const double radian);
  void MoveAir(const Eigen::Vector3d& translation);
  void RotateSky(const double radian);
  void ScaleAirFieldOfView(const int wheel);

  void PanoramaToAir();
  void AirToPanorama(const int panorama_index);
  
 private:
  bool Collide(const int from_index, const int to_index) const;
  void SetAirViewpoints(const Floorplan& floorplan);
  
  // Camera is at (center) and looks along (direction).
  CameraStatus camera_status;
  
  CameraPanorama camera_panorama;
  CameraAir camera_air;
  CameraBetweenPanoramaAndAir camera_between_panorama_and_air;
  CameraPanoramaTour camera_panorama_tour;

  // Z coordinate of the camera in the air.
  double air_height;
  // Angle of viewing in the air.
  double air_angle;
  // Field of view.
  double air_field_of_view_degrees;
  
  // Best ground_center for air.
  Eigen::Vector3d best_ground_center;
  Eigen::Vector3d best_start_directions_for_air[2];

  // Average distance.
  double average_distance;
  double average_floor_height;

  // Scaling in air field of view.
  double air_field_of_view_scale;

  const std::vector<PanoramaRenderer>& panorama_renderers;
  const PolygonRenderer& polygon_renderer;
  const std::map<int, int>& panorama_to_room;
  const std::map<int, int>& room_to_panorama;
};

}  // namespace structured_indoor_modeling

#endif  // NAVIGATION_H__
