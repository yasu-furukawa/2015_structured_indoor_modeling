#include <Eigen/Geometry>
#include <iostream>
#include "../base/floorplan.h"
#include "../base/panorama.h"
#include "configuration.h"
#include "navigation.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

  /*
Eigen::Vector3d ComputeAirDirectionFromPanorama(const Eigen::Vector3d& panorama_direction,
                                                const double air_height,
                                                const double air_angle) {
  Vector3d direction = panorama_direction.normalized();
  // Lift up by the angle.
  direction += -tan(air_angle)* Vector3d(0, 0, 1);
  direction *= air_height / tan(air_angle);

  return direction;
}
  */

Eigen::Vector3d ComputePanoramaDirectionFromAir(const Eigen::Vector3d& air_direction,
                                                const double distance) {
  Vector3d direction = air_direction;
  const int kZIndex = 2;
  direction[kZIndex] = 0.0;
  direction.normalize();
  direction *= distance;
  return direction;
}

Eigen::Vector3d ComputePanoramaDirectionFromFloorplan(const Eigen::Vector3d& floorplan_direction,
                                                      const double distance) {
  Vector3d direction = floorplan_direction;
  const int kZIndex = 2;
  direction[kZIndex] = 0.0;
  direction.normalize();
  direction *= distance;
  return direction;
}
  
void RobustifyDirection(const Eigen::Vector3d& lhs, Eigen::Vector3d* rhs) {
  const double length = (lhs.norm() + rhs->norm()) / 2.0;
  const double move_length = length * 0.01;

  Vector3d lhs_normalized = lhs.normalized();
  Vector3d rhs_normalized = rhs->normalized();

  const double kAlmostOpposite = -0.98;
  if (lhs_normalized.dot(rhs_normalized) <= kAlmostOpposite) {
    // Add a small offset.
    Vector3d orthogonal(lhs_normalized[1], lhs_normalized[0], 0.0);
    orthogonal.normalize();
    *rhs += orthogonal * move_length;
  }
}

}  // namespace

void CameraPanoramaTour::GetIndexWeightPairs(const double progress,
                                             int index_pair[2],
                                             int panorama_index_pair[2],
                                             double weight_pair[2]) const {
  const double index = progress * (indexes.size() - 1);
  index_pair[0] = static_cast<int>(floor(index));
  index_pair[1] = min(index_pair[0] + 1, (int)indexes.size() - 1);
  
  panorama_index_pair[0] = indexes[index_pair[0]];
  panorama_index_pair[1] = indexes[index_pair[1]];

  weight_pair[1] = index - index_pair[0];
  weight_pair[0] = 1.0 - weight_pair[1];
}

Eigen::Vector3d CameraPanoramaTour::GetCenter(const double progress) const {
  int index_pair[2];
  int panorama_index_pair[2];
  double weight_pair[2];
  GetIndexWeightPairs(progress, index_pair, panorama_index_pair, weight_pair);
  return weight_pair[0] * centers[index_pair[0]] + weight_pair[1] * centers[index_pair[1]];
}

Eigen::Vector3d CameraPanoramaTour::GetDirection(const double progress) const {
  int index_pair[2];
  int panorama_index_pair[2];
  double weight_pair[2];
  GetIndexWeightPairs(progress, index_pair, panorama_index_pair, weight_pair);
  Vector3d direction = weight_pair[0] * directions[index_pair[0]] + weight_pair[1] * directions[index_pair[1]];

  if (direction.norm() == 0.0) {
    cerr << "zero direction vector." << endl;
    exit (1);
  }
  
  return direction;
}

Navigation::Navigation(const Configuration& configuration,
                       const Floorplan& floorplan,
                       const vector<Panorama>& panoramas,
                       const std::map<int, int>& panorama_to_room,
                       const std::map<int, int>& room_to_panorama)
  : floorplan(floorplan), panoramas(panoramas),
    panorama_to_room(panorama_to_room), room_to_panorama(room_to_panorama) {
  air_angle = configuration.air_angle;
  air_field_of_view_degrees = configuration.air_field_of_view_degrees;
  floorplan_angle = configuration.floorplan_angle;
  floorplan_field_of_view_degrees = configuration.floorplan_field_of_view_degrees;
}

Vector3d Navigation::GetCenter() const {
  switch (camera_status) {
  case kPanorama: {
    return camera_panorama.start_center;
  }
  case kPanoramaTransition: {
    const double weight_start = ProgressInverse();
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
  case kFloorplan: {
    return camera_floorplan.GetCenter();
  }  
  case kFloorplanTransition: {
    const Vector3d direction = GetDirection();
    return camera_floorplan.ground_center - direction;
  }  
  case kPanoramaToAirTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_center +
      weight_end * (camera_in_transition.camera_air.GetCenter());
  }
  case kAirToPanoramaTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_air.GetCenter()) +
      weight_end * camera_in_transition.camera_panorama.start_center;
  }
  case kPanoramaToFloorplanTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_center +
      weight_end * (camera_in_transition.camera_floorplan.GetCenter());
  }
  case kFloorplanToPanoramaTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_floorplan.GetCenter()) +
      weight_end * camera_in_transition.camera_panorama.start_center;
  }
  case kAirToFloorplanTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_air.GetCenter()) +
      weight_end * (camera_in_transition.camera_floorplan.GetCenter());
  }
  case kFloorplanToAirTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_floorplan.GetCenter()) +
      weight_end * (camera_in_transition.camera_air.GetCenter());
  }    
  case kPanoramaTour: {
    return camera_panorama_tour.GetCenter(1.0 - ProgressInverse());
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
    const Vector3d normalized_direction = direction.normalized();
    if (normalized_direction.norm() == 0.0) {
      cout << "Internal zero " << camera_panorama.start_direction << endl
           << camera_panorama.end_direction << endl
           << weight_start << ' ' << weight_end << endl;
      exit (1);
    }
    if (std::isnan(normalized_direction[0]) ||
        std::isnan(normalized_direction[1]) ||
        std::isnan(normalized_direction[2])) {
      cout << "Internal nan " << camera_panorama.start_direction << endl
           << camera_panorama.end_direction << endl
           << weight_start << ' ' << weight_end << endl;
      exit (1);
    }
    
    return normalized_direction;
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
  }
  case kFloorplan: {
    return camera_floorplan.start_direction;
  }
  case kFloorplanTransition: {
    Vector3d horizontal_direction = camera_floorplan.start_direction;
    const double vertical_distance = horizontal_direction[2];
    horizontal_direction[2] = 0.0;
    const double horizontal_distance = horizontal_direction.norm();
    
    const double weight_start = (cos(camera_floorplan.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    Vector3d direction = weight_start * camera_floorplan.start_direction +
      weight_end * camera_floorplan.end_direction;
    direction[2] = 0.0;
    direction.normalize();
    direction *= horizontal_distance;
    direction[2] = vertical_distance;
    return direction;
  }
    
  case kPanoramaToAirTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_direction +
      weight_end * camera_in_transition.camera_air.start_direction;
  }
  case kAirToPanoramaTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_air.start_direction +
      weight_end * camera_in_transition.camera_panorama.start_direction;
  }
  case kPanoramaToFloorplanTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_direction +
      weight_end * camera_in_transition.camera_floorplan.start_direction;
  }
  case kFloorplanToPanoramaTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_floorplan.start_direction +
      weight_end * camera_in_transition.camera_panorama.start_direction;
  }
  case kAirToFloorplanTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_air.start_direction +
      weight_end * camera_in_transition.camera_floorplan.start_direction;
  }
  case kFloorplanToAirTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_floorplan.start_direction +
      weight_end * camera_in_transition.camera_air.start_direction;
  }
    
  case kPanoramaTour: {
    return camera_panorama_tour.GetDirection(1.0 - ProgressInverse());
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

const CameraFloorplan& Navigation::GetCameraFloorplan() const {
  return camera_floorplan;
}
  
const CameraInTransition& Navigation::GetCameraInTransition() const {
  return camera_in_transition;
}

const CameraPanoramaTour& Navigation::GetCameraPanoramaTour() const {
  return camera_panorama_tour;
}

void Navigation::Init() {
  if (panoramas.empty()) {
    cerr << "No panoramas." << endl;
    exit (1);
  }

  const int kStartIndex = 0;
  camera_status = kPanorama;
  camera_panorama.start_index = kStartIndex;
  camera_panorama.start_center = panoramas[kStartIndex].GetCenter();
  camera_panorama.start_direction =
    floorplan.GetRoomCenterGlobal(kStartIndex) - camera_panorama.start_center;
  camera_panorama.start_direction[2] = 0.0;
  camera_panorama.start_direction.normalize();
  camera_panorama.start_direction *= panoramas[kStartIndex].GetAverageDistance();
  camera_panorama.progress = 0.0;

  {
    average_distance = 0.0;
    for (const auto& panorama : panoramas) {
      average_distance += panorama.GetAverageDistance();
    }
    average_distance /= static_cast<int>(panoramas.size());
  }

  SetAirFloorplanViewpoints(floorplan);
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
  case kFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_floorplan.progress += kStepSize;
    if (camera_floorplan.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan.start_direction = camera_floorplan.end_direction;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kPanoramaToAirTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kAir;

      camera_air = camera_in_transition.camera_air;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kAirToPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kPanoramaTransition;
      camera_panorama = camera_in_transition.camera_panorama;
      camera_panorama.end_index = camera_panorama.start_index;
      camera_panorama.end_center = camera_panorama.start_center;

      const int room = panorama_to_room.find(camera_panorama.start_index)->second;
      camera_panorama.end_direction =
        floorplan.GetRoomCenterGlobal(room) - camera_panorama.start_center;
      camera_panorama.end_direction[2] = 0.0;
      camera_panorama.end_direction.normalize();
      camera_panorama.end_direction *= panoramas[camera_panorama.start_index].GetAverageDistance();
      RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);

      camera_panorama.progress = 0.0;
    }
    break;
  }
  case kPanoramaToFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan = camera_in_transition.camera_floorplan;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kFloorplanToPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kPanoramaTransition;
      camera_panorama = camera_in_transition.camera_panorama;
      camera_panorama.end_index = camera_panorama.start_index;
      camera_panorama.end_center = camera_panorama.start_center;

      const int room = panorama_to_room.find(camera_panorama.start_index)->second;
      camera_panorama.end_direction =
        floorplan.GetRoomCenterGlobal(room) - camera_panorama.start_center;
      camera_panorama.end_direction[2] = 0.0;
      camera_panorama.end_direction.normalize();
      camera_panorama.end_direction *= panoramas[camera_panorama.start_index].GetAverageDistance();
      RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);

      camera_panorama.progress = 0.0;
    }
    break;
  }
  case kAirToFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan = camera_in_transition.camera_floorplan;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kFloorplanToAirTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kAir;

      camera_air = camera_in_transition.camera_air;
      camera_air.progress = 0.0;
    }
    break;
  }
    
  case kPanoramaTour: {
    double step_size;
    if (camera_panorama_tour.indexes.size() > 4)
      step_size = 0.01;
    else
      step_size = 0.02;
    camera_panorama_tour.progress += step_size;
    if (camera_panorama_tour.progress >= 1.0) {
      camera_status = kPanoramaTransition;
      camera_panorama.start_index     = camera_panorama_tour.indexes.back();
      camera_panorama.start_center    = camera_panorama_tour.centers.back();
      camera_panorama.start_direction = camera_panorama_tour.directions.back();
      camera_panorama.end_index     = camera_panorama.start_index;
      camera_panorama.end_center    = camera_panorama.start_center;
      const int room = panorama_to_room.find(camera_panorama.start_index)->second;
      camera_panorama.end_direction = floorplan.GetRoomCenterGlobal(room) - camera_panorama.start_center;
      camera_panorama.end_direction[2] = 0.0;
      camera_panorama.end_direction.normalize();
      camera_panorama.end_direction *= panoramas[camera_panorama.start_index].GetAverageDistance();
      RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);
      
      camera_panorama.progress = 0.0;
    }
    break;
  }
  default: {
    break;
  }
  }
}

void Navigation::RotatePanorama(const double dx, const double dy) {
  Vector3d zaxis = camera_panorama.start_direction;
  Vector3d yaxis(0, 0, 1);
  Vector3d xaxis = yaxis.cross(zaxis);
  yaxis = zaxis.cross(xaxis);

  if (dx == 0 && dy == 0)
    return;

  xaxis.normalize();
  yaxis.normalize();

  Matrix3d rotation;
  if (dx == 0) {
    rotation = AngleAxisd(-dy, xaxis).toRotationMatrix();
  } else if (dy == 0) {
    rotation = AngleAxisd(dx, yaxis).toRotationMatrix();
  } else {
    rotation = AngleAxisd(-dy, xaxis).toRotationMatrix() * AngleAxisd(dx, yaxis).toRotationMatrix();
  }

  camera_panorama.start_direction = rotation * camera_panorama.start_direction;
}

void Navigation::MoveAir(const Eigen::Vector3d& translation)  {
  camera_air.ground_center += translation;
}

void Navigation::MoveFloorplan(const Eigen::Vector3d& translation)  {
  camera_floorplan.ground_center += translation;
}

bool Navigation::Collide(const int /*from_index*/, const int /*to_index*/) const {
  return false;
}

void Navigation::SetAirFloorplanViewpoints(const Floorplan& floorplan) { 
  // Compute best ground_center and start_direction for air.  
  Eigen::Vector2d x_range, y_range;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
      const Eigen::Vector2d local = floorplan.GetRoomVertexLocal(room, v);
      if (room == 0 && v == 0) {
        x_range[0] = x_range[1] = local[0];
        y_range[0] = y_range[1] = local[1];
      } else {
        x_range[0] = min(x_range[0], local[0]);
        x_range[1] = max(x_range[1], local[0]);
        y_range[0] = min(y_range[0], local[1]);
        y_range[1] = max(y_range[1], local[1]);
      }
    }
  }

  //----------------------------------------------------------------------
  {
    air_floorplan_field_of_view_scale = 1.0;

    // diameter must be visible in the given field-of-view along air_angle.
    const double diameter = max(x_range[1] - x_range[0], y_range[1] - y_range[0]);
    air_height = diameter / 2.0 / tan(air_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(air_angle);
    air_height *= 0.8;

    floorplan_height = diameter / 2.0 / tan(floorplan_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(floorplan_angle);
    floorplan_height *= 0.8;
  }

  const Vector2d center_local((x_range[0] + x_range[1]) / 2.0,
                              (y_range[0] + y_range[1]) / 2.0);

  average_floor_height = 0.0;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    average_floor_height += floorplan.GetFloorHeight(room);
  }
  average_floor_height /= floorplan.GetNumRooms();

  best_ground_center =
    floorplan.GetFloorplanToGlobal() *
    Vector3d(center_local[0], center_local[1], average_floor_height);

  for (int i = 0; i < 2; ++i) {
    // Y axis is the viewing direction.
    if ((x_range[1] - x_range[0]) > (y_range[1] - y_range[0])) {
      if (i == 0)
        best_start_directions_for_air[i] = 
          floorplan.GetFloorplanToGlobal() * Vector3d(0, 1, 0);
      else
        best_start_directions_for_air[i] = 
          floorplan.GetFloorplanToGlobal() * Vector3d(0, -1, 0);
    } else {
      if (i == 0)
        best_start_directions_for_air[i] = 
          floorplan.GetFloorplanToGlobal() * Vector3d(1, 0, 0);
      else
        best_start_directions_for_air[i] = 
          floorplan.GetFloorplanToGlobal() * Vector3d(-1, 0, 0);
    }
    best_start_directions_for_floorplan[i] = best_start_directions_for_air[i];
        
    best_start_directions_for_air[i] += -tan(air_angle) * Vector3d(0, 0, 1);
    best_start_directions_for_air[i] *= air_height / tan(air_angle);

    best_start_directions_for_floorplan[i] += -tan(floorplan_angle) * Vector3d(0, 0, 1);
    best_start_directions_for_floorplan[i] *= floorplan_height / tan(floorplan_angle);
  }
}  

void Navigation::MoveToPanorama(const int target_panorama_index) {
  cerr << "Move to " << target_panorama_index << endl;
  camera_panorama.end_index = target_panorama_index;
  camera_panorama.end_center = panoramas[target_panorama_index].GetCenter();

  Vector3d movement = panoramas[target_panorama_index].GetCenter() -
    panoramas[camera_panorama.start_index].GetCenter();
  // movement.normalize();

  Vector3d sum;
  if (camera_panorama.start_direction.dot(movement) > 0.0)
    sum = camera_panorama.start_direction + movement;
  else
    sum = camera_panorama.start_direction - movement;
  // Makes it more horizontal.
  sum[2] /= 2.0;
  sum.normalize();
  camera_panorama.end_direction =
    sum * panoramas[target_panorama_index].GetAverageDistance();

  RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);
  
  // Starts animation.
  camera_panorama.progress = 0.0;
  camera_status = kPanoramaTransition;
}

void Navigation::TourToPanorama(const std::vector<int>& indexes) {
  // Find a sequence of paths from camera_panorama.start_index to target_panorama_index.
  camera_panorama_tour.progress = 0.0;
  camera_panorama_tour.indexes = indexes;
  camera_panorama_tour.centers.resize(indexes.size());
  camera_panorama_tour.directions.resize(indexes.size());
  for (int i = 0; i < (int)indexes.size(); ++i) {
    camera_panorama_tour.centers[i] = panoramas[indexes[i]].GetCenter();
  }

  camera_panorama_tour.directions[0] = camera_panorama.start_direction;
  for (int i = 1; i < (int)indexes.size(); ++i) {
    const Vector3d movement = camera_panorama_tour.centers[i] -
      camera_panorama_tour.centers[i - 1];
    
    camera_panorama_tour.directions[i] =
      camera_panorama_tour.directions[i - 1] + movement;
    camera_panorama_tour.directions[i].normalize();
    camera_panorama_tour.directions[i] *= panoramas[indexes[i]].GetAverageDistance();
  }
  
  camera_status = kPanoramaTour;
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

  for (int p = 0; p < static_cast<int>(panoramas.size()); ++p) {
    if (p == camera_panorama.start_index)
      continue;
    // Behind.
    const Vector3d diff = panoramas[p].GetCenter() - camera_panorama.start_center;
    
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
  // const int target_panorama_index = (camera_panorama.start_index + 1) % panoramas.size();
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

  RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);
}

void Navigation::RotateAir(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_air.end_direction = rotation * camera_air.start_direction;
  camera_air.progress = 0.0;
  camera_status = kAirTransition;  
}

void Navigation::RotateFloorplan(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_floorplan.end_direction = rotation * camera_floorplan.start_direction;
  camera_floorplan.progress = 0.0;
  camera_status = kFloorplanTransition;  
}
  
void Navigation::ScaleAirFloorplanFieldOfView(const int wheel) {
  air_floorplan_field_of_view_scale += wheel / 800.0;
  
  air_floorplan_field_of_view_scale = max(air_floorplan_field_of_view_scale, 0.25);
  air_floorplan_field_of_view_scale = min(air_floorplan_field_of_view_scale, 4.0);
}

void Navigation::PanoramaToAir() {
  air_floorplan_field_of_view_scale = 1.0;
  
  camera_status = kPanoramaToAirTransition;

  camera_in_transition.camera_panorama = camera_panorama;
  {
    CameraAir& camera_air      = camera_in_transition.camera_air;
    camera_air.ground_center = best_ground_center;
    if (camera_panorama.start_direction.dot(best_start_directions_for_air[0]) >
        camera_panorama.start_direction.dot(best_start_directions_for_air[1]))
      camera_air.start_direction = best_start_directions_for_air[0];
    else
      camera_air.start_direction = best_start_directions_for_air[1];
  }
  
  camera_in_transition.progress = 0.0;  
}

void Navigation::AirToPanorama(const int panorama_index) {
  camera_status = kAirToPanoramaTransition;
  
  camera_in_transition.camera_air = camera_air;
  {
    CameraPanorama& camera_panorama =
      camera_in_transition.camera_panorama;
    camera_panorama.start_index     = panorama_index;
    camera_panorama.start_center    = panoramas[panorama_index].GetCenter();
    camera_panorama.start_direction =
      ComputePanoramaDirectionFromAir(camera_air.start_direction,
                                      panoramas[panorama_index].GetAverageDistance());
  }
  camera_in_transition.progress = 0.0;  
}

void Navigation::PanoramaToFloorplan() {
  air_floorplan_field_of_view_scale = 1.0;

  camera_status = kPanoramaToFloorplanTransition;

  camera_in_transition.camera_panorama = camera_panorama;
  {
    CameraFloorplan& camera_floorplan      = camera_in_transition.camera_floorplan;
    camera_floorplan.ground_center = best_ground_center;
    if (camera_panorama.start_direction.dot(best_start_directions_for_floorplan[0]) >
        camera_panorama.start_direction.dot(best_start_directions_for_floorplan[1]))
      camera_floorplan.start_direction = best_start_directions_for_floorplan[0];
    else
      camera_floorplan.start_direction = best_start_directions_for_floorplan[1];
  }
  
  camera_in_transition.progress = 0.0;  
}

void Navigation::FloorplanToPanorama(const int panorama_index) {
  camera_status = kFloorplanToPanoramaTransition;
  
  camera_in_transition.camera_floorplan = camera_floorplan;
  {
    CameraPanorama& camera_panorama =
      camera_in_transition.camera_panorama;
    camera_panorama.start_index     = panorama_index;
    camera_panorama.start_center    = panoramas[panorama_index].GetCenter();
    camera_panorama.start_direction =
      ComputePanoramaDirectionFromFloorplan(camera_floorplan.start_direction,
                                            panoramas[panorama_index].GetAverageDistance());
  }
  camera_in_transition.progress = 0.0;  
}

void Navigation::AirToFloorplan() {
  camera_status = kAirToFloorplanTransition;

  camera_in_transition.camera_air = camera_air;
  {
    CameraFloorplan& camera_floorplan      = camera_in_transition.camera_floorplan;
    camera_floorplan.ground_center = camera_air.ground_center;
    if (camera_air.start_direction.dot(best_start_directions_for_floorplan[0]) >
        camera_air.start_direction.dot(best_start_directions_for_floorplan[1]))
      camera_floorplan.start_direction = best_start_directions_for_floorplan[0];
    else
      camera_floorplan.start_direction = best_start_directions_for_floorplan[1];
  }
  
  camera_in_transition.progress = 0.0;  
}

void Navigation::FloorplanToAir() {
  camera_status = kFloorplanToAirTransition;

  camera_in_transition.camera_floorplan = camera_floorplan;
  {
    CameraAir& camera_air      = camera_in_transition.camera_air;
    camera_air.ground_center = camera_floorplan.ground_center;
    if (camera_floorplan.start_direction.dot(best_start_directions_for_air[0]) >
        camera_floorplan.start_direction.dot(best_start_directions_for_air[1]))
      camera_air.start_direction = best_start_directions_for_air[0];
    else
      camera_air.start_direction = best_start_directions_for_air[1];
  }
  
  camera_in_transition.progress = 0.0;  
}
  
double Navigation::ProgressInverse() const {
  switch (camera_status) {
  case kPanoramaTransition:
    return cos(camera_panorama.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kAirTransition:
    return cos(camera_air.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kFloorplanTransition:
    return cos(camera_floorplan.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition:
  case kPanoramaToFloorplanTransition:
  case kFloorplanToPanoramaTransition:
  case kAirToFloorplanTransition:
  case kFloorplanToAirTransition:
    {
    return cos(camera_in_transition.progress * M_PI) / 2.0 + 1.0 / 2.0;
    // sigmoid.
    /*
    const double minimum_value = 1 / (1 + exp(6.0));
    const double maximum_value = 1 / (1 + exp(-6.0));
    const double sigmoid = 1 / (1 + exp(- (6.0 - 12.0 * camera_in_transition.progress)));
    return (sigmoid - minimum_value) / (maximum_value - minimum_value);
    */
    //return 1.0 - sin(camera_in_transition.progress * M_PI / 2);
  }
  case kPanoramaTour:
    return cos(camera_panorama_tour.progress * M_PI) / 2.0 + 1.0 / 2.0;
  default:
    cerr << "Impossible in ProgressInverse." << endl;
    exit (1);
  }
}

double Navigation::GetFieldOfViewInDegrees() const {
  const double kPanoramaFieldOfViewDegrees = 100.0;
  const double scaled_air_field_of_view_degrees =
    air_field_of_view_degrees * air_floorplan_field_of_view_scale;
  const double scaled_floorplan_field_of_view_degrees =
    floorplan_field_of_view_degrees * air_floorplan_field_of_view_scale;

  switch (camera_status) {
  case kPanorama:
  case kPanoramaTransition:
  case kPanoramaTour: {
    return kPanoramaFieldOfViewDegrees;
  }
  // CameraAir handles the state.
  case kAir:
  case kAirTransition: {
    return scaled_air_field_of_view_degrees;
  }
  case kFloorplan:
  case kFloorplanTransition: {
    return scaled_floorplan_field_of_view_degrees;
  }
  // CameraInTransition handles the state.
  case kPanoramaToAirTransition: {    
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_panorama.start_center[2],
                                               camera_in_transition.camera_air.GetCenter()[2],
                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0,
                                               scaled_air_field_of_view_degrees * M_PI / 180.0);
  }
  case kAirToPanoramaTransition: {
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_air.GetCenter()[2],
                                               camera_in_transition.camera_panorama.start_center[2],
                                               scaled_air_field_of_view_degrees * M_PI / 180.0,
                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0);
  }
  case kPanoramaToFloorplanTransition: {
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_panorama.start_center[2],
                                               camera_in_transition.camera_floorplan.GetCenter()[2],
                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0,
                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0);
  }
  case kFloorplanToPanoramaTransition: {
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_floorplan.GetCenter()[2],
                                               camera_in_transition.camera_panorama.start_center[2],
                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0,
                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0);
  }
  case kAirToFloorplanTransition: {
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_air.GetCenter()[2],
                                               camera_in_transition.camera_floorplan.GetCenter()[2],
                                               scaled_air_field_of_view_degrees * M_PI / 180.0,
                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0);
  }
  case kFloorplanToAirTransition: {
    return GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_floorplan.GetCenter()[2],
                                               camera_in_transition.camera_air.GetCenter()[2],
                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0,
                                               scaled_air_field_of_view_degrees * M_PI / 180.0);
  }    
  }
}

double Navigation::GetFieldOfViewInTransitionInDegrees(const double start_height,
                                                       const double end_height,
                                                       const double start_field_of_view,
                                                       const double end_field_of_view) const {
  const double start_height_diff = start_height - average_floor_height;
  const double end_height_diff   = end_height - average_floor_height;

  const double start_size = 1.0 / tan(start_field_of_view / 2.0) / start_height_diff;
  const double end_size   = 1.0 / tan(end_field_of_view / 2.0) / end_height_diff;

  const double start_weight = ProgressInverse();
  const double end_weight = 1.0 - start_weight;

  const double current_height_diff = GetCenter()[2] - average_floor_height;
  const double current_size = start_weight * start_size + end_weight * end_size;
  const double current_field_of_view = atan(1.0 / current_height_diff / current_size) * 2.0;

  return current_field_of_view * 180.0 / M_PI; 
}

}  // namespace structured_indoor_modeling
