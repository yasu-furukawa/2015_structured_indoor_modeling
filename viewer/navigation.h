#ifndef NAVIGATION_H__
#define NAVIGATION_H__

#include <Eigen/Dense>
#include <map>
#include <vector>

namespace structured_indoor_modeling {

class Floorplan;
class Panorama;
class ViewParameters;

enum CameraStatus {
  // CameraPanorama handles the states.
  kPanorama,
  kPanoramaTransition,
  // CameraAir handles the state.
  kAir,
  kAirTransition,
  // CameraFloorplan handles the state.
  kFloorplan,
  kFloorplanTransition,
  // CameraInTransition handles the states.
  kPanoramaToAirTransition,
  kAirToPanoramaTransition,

  kPanoramaToFloorplanTransition,
  kFloorplanToPanoramaTransition,

  kAirToFloorplanTransition,
  kFloorplanToAirTransition,

  // Tour
  kPanoramaTour,

  // Tree view.
  kTree,
  kTreeTransition,
  kTreeToAirTransition,
  kAirToTreeTransition
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

struct CameraFloorplan {
  Eigen::Vector3d ground_center;
  Eigen::Vector3d start_direction;
  Eigen::Vector3d end_direction;

  double progress;

  Eigen::Vector3d GetCenter() const {
    return ground_center - start_direction;
  }
};

struct CameraInTransition {
  // "progress" is not used in camera_panorama and camera_air.
  CameraPanorama camera_panorama;
  CameraAir camera_air;
  CameraFloorplan camera_floorplan;
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
  Navigation(const Floorplan& floorplan,
             const ViewParameters& view_parameters,
             const std::vector<Panorama>& panoramas,
             const std::map<int, int>& panorama_to_room,
             const std::map<int, int>& room_to_panorama);

  // Accessors.
  Eigen::Vector3d GetCenter() const;
  Eigen::Vector3d GetDirection() const;
  CameraStatus GetCameraStatus() const;
  const CameraPanorama& GetCameraPanorama() const;
  const CameraAir& GetCameraAir() const;
  const CameraFloorplan& GetCameraFloorplan() const;
  const CameraInTransition& GetCameraInTransition() const;
  const CameraPanoramaTour& GetCameraPanoramaTour() const;

  // double Progress() const;
  double ProgressInverse() const;
  double GetFieldOfViewInDegrees() const;
  double GetTreeProgress() const { return tree_progress; }

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
  void MoveFloorplan(const Eigen::Vector3d& translation);
  void RotateAir(const double radian);
  void RotateFloorplan(const double radian);
  void ScaleAirFloorplanFieldOfView(const int wheel);

  void PanoramaToAir();
  void AirToPanorama(const int panorama_index);
  void PanoramaToFloorplan();
  void FloorplanToPanorama(const int panorama_index);
  void AirToFloorplan();
  void FloorplanToAir();
  void AirToTree();
  void TreeToAir();
  
 private:
  bool Collide(const int from_index, const int to_index) const;
  double GetFieldOfViewInTransitionInDegrees(const double start_height,
                                             const double end_height,
                                             const double start_field_of_view,
                                             const double end_field_of_view) const;
  
  // Camera is at (center) and looks along (direction).
  CameraStatus camera_status;
  
  CameraPanorama camera_panorama;
  CameraAir camera_air;
  CameraFloorplan camera_floorplan;
  CameraInTransition camera_in_transition;
  CameraPanoramaTour camera_panorama_tour;

  double tree_progress;

  // Scaling in air field of view.
  double air_floorplan_field_of_view_scale;

  const Floorplan& floorplan;
  const ViewParameters& view_parameters;
  const std::vector<Panorama>& panoramas;
  const std::map<int, int>& panorama_to_room;
  // const std::map<int, int>& room_to_panorama;
};

}  // namespace structured_indoor_modeling

#endif  // NAVIGATION_H__
