#include <limits>
#include <numeric>
#include "view_parameters.h"
#include "configuration.h"
#include "../base/panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ViewParameters::ViewParameters(const Floorplan& floorplan,
                               const IndoorPolygon& indoor_polygon,
                               const ObjectRenderer& object_renderer,
                               const std::vector<Panorama>& panoramas,
                               const Configuration& configuration) :
  floorplan(floorplan),
  indoor_polygon(indoor_polygon),
  object_renderer(object_renderer),
  panoramas(panoramas),
  air_angle(configuration.air_angle),
  floorplan_angle(configuration.floorplan_angle),
  air_field_of_view_degrees(configuration.air_field_of_view_degrees),
  floorplan_field_of_view_degrees(configuration.floorplan_field_of_view_degrees) {
}

void ViewParameters::Init() {
  room_configurations.resize(floorplan.GetNumRooms());
  floor_configurations.resize(floorplan.GetNumRooms());
  wall_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    wall_configurations[room].resize(floorplan.GetNumWalls(room));
  object_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    object_configurations[room].resize(object_renderer.GetNumObjects(room));

  InitAirFloorplanViewpoints();
  
  InitCenter();
  InitBoundingBoxes();
  InitTreeConfigurationCenter();
  SetDisplacements();
}

void ViewParameters::InitAirFloorplanViewpoints() {
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
    // diameter must be visible in the given field-of-view along air_angle.
    const double diameter = max(x_range[1] - x_range[0], y_range[1] - y_range[0]);
    //?????
    air_height = diameter / 2.0 / tan(air_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(air_angle);
    // air_height *= 0.75;

    floorplan_height = diameter / 2.0 / tan(floorplan_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(floorplan_angle);
    // floorplan_height *= 0.75;
  }

  
  const Vector2d center_local((x_range[0] + x_range[1]) / 2.0,
                              (y_range[0] + y_range[1]) / 2.0);

  average_floor_height = 0.0;
  average_ceiling_height = 0.0;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    average_floor_height += floorplan.GetFloorHeight(room);
    average_ceiling_height += floorplan.GetCeilingHeight(room);
  }
  average_floor_height /= floorplan.GetNumRooms();
  average_ceiling_height /= floorplan.GetNumRooms();

  {
    average_distance = 0.0;
    for (const auto& panorama : panoramas) {
      average_distance += panorama.GetAverageDistance();
    }
    average_distance /= static_cast<int>(panoramas.size());
  }
  
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

  if ((x_range[1] - x_range[0]) > (y_range[1] - y_range[0])) {
    x_axis = floorplan.GetFloorplanToGlobal() * Vector3d(1, 0, 0);
    y_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 1, 0);    
    z_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 0, 1);
  } else {
    x_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 1, 0);
    y_axis = floorplan.GetFloorplanToGlobal() * Vector3d(1, 0, 0);    
    z_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 0, 1);
  }
}    
  
void ViewParameters::InitCenter() {
  center = Vector3d(0, 0, 0);
  BoundingBox bounding_box;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
      const Eigen::Vector3d floor_vertex = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, v));
      for (int a = 0; a < 3; ++a) {
        bounding_box.min_xyz[a] = min(bounding_box.min_xyz[a], floor_vertex[a]);
        bounding_box.max_xyz[a] = max(bounding_box.max_xyz[a], floor_vertex[a]);
      }
      const Eigen::Vector3d ceiling_vertex = GlobalToLocal(floorplan.GetCeilingVertexGlobal(room, v));
      for (int a = 0; a < 3; ++a) {
        bounding_box.min_xyz[a] = min(bounding_box.min_xyz[a], ceiling_vertex[a]);
        bounding_box.max_xyz[a] = max(bounding_box.max_xyz[a], ceiling_vertex[a]);
      }
    }
  }

  center = LocalToGlobal((bounding_box.min_xyz + bounding_box.max_xyz) / 2.0);
}

void ViewParameters::InitBoundingBoxes() {
  //----------------------------------------------------------------------
  // room_configurations.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
      const Eigen::Vector3d floor_vertex = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, v));
      for (int a = 0; a < 3; ++a) {
        room_configurations[room].bounding_box.min_xyz[a] =
          min(room_configurations[room].bounding_box.min_xyz[a], floor_vertex[a]);
        room_configurations[room].bounding_box.max_xyz[a] =
          max(room_configurations[room].bounding_box.max_xyz[a], floor_vertex[a]);
      }
      const Eigen::Vector3d ceiling_vertex = GlobalToLocal(floorplan.GetCeilingVertexGlobal(room, v));
      for (int a = 0; a < 3; ++a) {
        room_configurations[room].bounding_box.min_xyz[a] =
          min(room_configurations[room].bounding_box.min_xyz[a], ceiling_vertex[a]);
        room_configurations[room].bounding_box.max_xyz[a] =
          max(room_configurations[room].bounding_box.max_xyz[a], ceiling_vertex[a]);
      }
    }
  }

  //----------------------------------------------------------------------
  // floor_configurations.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
      const Eigen::Vector3d floor_vertex = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, v));
      for (int a = 0; a < 3; ++a) {
        floor_configurations[room].bounding_box.min_xyz[a] =
          min(floor_configurations[room].bounding_box.min_xyz[a], floor_vertex[a]);
        floor_configurations[room].bounding_box.max_xyz[a] =
          max(floor_configurations[room].bounding_box.max_xyz[a], floor_vertex[a]);
      }
    }
  }

  //----------------------------------------------------------------------
  // wall_configurations.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
      const int next_v = (v + 1) % floorplan.GetNumRoomVertices(room);
      const Eigen::Vector3d floor0 = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, v));
      const Eigen::Vector3d floor1 = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, next_v));
      const Eigen::Vector3d ceiling0 = GlobalToLocal(floorplan.GetCeilingVertexGlobal(room, v));
      const Eigen::Vector3d ceiling1 = GlobalToLocal(floorplan.GetCeilingVertexGlobal(room, next_v));
      
      for (int a = 0; a < 3; ++a) {
        wall_configurations[room][v].bounding_box.min_xyz[a] =
          min(min(floor0[a], floor1[a]), min(ceiling0[a], ceiling1[a]));
        wall_configurations[room][v].bounding_box.max_xyz[a] =
          max(max(floor0[a], floor1[a]), max(ceiling0[a], ceiling1[a]));
      }
    }
  }

  //----------------------------------------------------------------------
  // object_configurations.
  for (int room = 0; room < object_renderer.GetNumRooms(); ++room) {
    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const vector<float>& points = object_renderer.GetObject(room, object);
      for (int v = 0; v < (int)points.size(); v += 3) {
        const Vector3d point = GlobalToLocal(Vector3d(points[v], points[v + 1], points[v + 2]));
        for (int a = 0; a < 3; ++a) {
          object_configurations[room][object].bounding_box.min_xyz[a] =
            min(object_configurations[room][object].bounding_box.min_xyz[a], point[a]);
          object_configurations[room][object].bounding_box.max_xyz[a] =
            max(object_configurations[room][object].bounding_box.max_xyz[a], point[a]);
        }
      }
    }
  }
}

void ViewParameters::InitTreeConfigurationCenter() {
  for (auto& configuration : room_configurations) {
    configuration.center =
      (configuration.bounding_box.min_xyz + configuration.bounding_box.max_xyz) / 2.0;
  }

  for (auto& configuration : floor_configurations) {
    configuration.center =
      (configuration.bounding_box.min_xyz + configuration.bounding_box.max_xyz) / 2.0;
  }

  for (auto& configurations : wall_configurations) {
    for (auto& configuration : configurations) {
      configuration.center =
        (configuration.bounding_box.min_xyz + configuration.bounding_box.max_xyz) / 2.0;
    }
  }

  for (auto& configurations : object_configurations) {
    for (auto& configuration : configurations) {
      configuration.center =
        (configuration.bounding_box.min_xyz + configuration.bounding_box.max_xyz) / 2.0;
    }
  }
}

void ViewParameters::SetDisplacements() {
  // To allow some margin between items.
  const double kShrinkScale = 0.95;
  vector<double> room_sizes(floorplan.GetNumRooms());
  vector<pair<double, int> > room_offsets(floorplan.GetNumRooms());
  Vector2d x_range(numeric_limits<double>::max(), - numeric_limits<double>::max());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const BoundingBox& bounding_box = room_configurations[room].bounding_box;
    room_sizes[room] = Vector2d(bounding_box.max_xyz[0] - bounding_box.min_xyz[0],
                                bounding_box.max_xyz[1] - bounding_box.min_xyz[1]).norm();
    room_offsets[room] = pair<double, int>(room_configurations[room].center[0], room);
    
    x_range[0] = min(x_range[0], bounding_box.min_xyz[0]);
    x_range[1] = max(x_range[1], bounding_box.max_xyz[0]);
  }
  sort(room_offsets.begin(), room_offsets.end());

  //----------------------------------------------------------------------
  // Room displacements.
  
  const double room_size_sum = accumulate(room_sizes.begin(), room_sizes.end(), 0.0);
  const double scale = (x_range[1] - x_range[0]) / room_size_sum;

  double accumulated_position = - room_size_sum / 2.0;

  for (int i = 0; i < (int)room_offsets.size(); ++i) {
    const int room = room_offsets[i].second;
    const double position = accumulated_position + room_sizes[room] / 2.0;
    room_configurations[room].displacement =
      Vector3d(position * scale, 0, 0) - room_configurations[room].center;
    
    room_configurations[room].scale = kShrinkScale * scale;
    accumulated_position += room_sizes[room];
  }
  //----------------------------------------------------------------------
  // Floor and Wall displacements are 0 for the moment.

  //----------------------------------------------------------------------
  // Object displacements.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    vector<double> object_sizes(object_renderer.GetNumObjects(room));
    vector<pair<double, int> > object_offsets(object_renderer.GetNumObjects(room));
    const Eigen::Vector3d& reference = room_configurations[room].center;
    const Vector2d x_range(scale * (room_configurations[room].bounding_box.min_xyz[0] - reference[0]),
                           scale * (room_configurations[room].bounding_box.max_xyz[0] - reference[0]));

    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const BoundingBox& bounding_box = object_configurations[room][object].bounding_box;
      object_sizes[object] = Vector2d(bounding_box.max_xyz[0] - bounding_box.min_xyz[0],
                                      bounding_box.max_xyz[1] - bounding_box.min_xyz[1]).norm();
      object_offsets[object] =
        pair<double, int>(object_configurations[room][object].center[0] - reference[0], object);
    }
    sort(object_offsets.begin(), object_offsets.end());

    const double object_size_sum = accumulate(object_sizes.begin(), object_sizes.end(), 0.0);
    const double kDefaultShrink = 0.5;
    const int num_rows = min(2, static_cast<int>(ceil(kDefaultShrink * object_size_sum / round(x_range[1] - x_range[0]))));

    const int num_items_per_row =
      static_cast<int>(ceil(object_renderer.GetNumObjects(room) / (double)num_rows));
    for (int row = 0; row < num_rows; ++row) {
      const int start_index = num_items_per_row * row;
      const int end_index = min(object_renderer.GetNumObjects(room), num_items_per_row * (row + 1));

      double object_size_sum_per_row = 0.0;
      for (int i = start_index; i < end_index; ++i)
        object_size_sum_per_row += object_sizes[object_offsets[i].second];
      
      const double object_scale = min(1.0, (x_range[1] - x_range[0]) / object_size_sum_per_row);
      double accumulated_position = - object_size_sum_per_row / 2.0;
      for (int i = start_index; i < end_index; ++i) {
        const int object = object_offsets[i].second;
        const double position = accumulated_position + object_sizes[object] / 2.0;
        object_configurations[room][object].displacement =
          Vector3d(position * object_scale, 0.0, 0.0) -
          (object_configurations[room][object].center - reference);
        
        object_configurations[room][object].scale = kShrinkScale * object_scale;
        object_configurations[room][object].row = row;
        accumulated_position += object_sizes[object];
      }
    }
  }
  
  /*
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    vector<double> object_sizes(object_renderer.GetNumObjects(room));
    vector<pair<double, int> > object_offsets(object_renderer.GetNumObjects(room));
    const Eigen::Vector3d& reference = room_configurations[room].center;
    const Vector2d x_range(scale * (room_configurations[room].bounding_box.min_xyz[0] - reference[0]),
                           scale * (room_configurations[room].bounding_box.max_xyz[0] - reference[0]));

    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const BoundingBox& bounding_box = object_configurations[room][object].bounding_box;
      object_sizes[object] = Vector2d(bounding_box.max_xyz[0] - bounding_box.min_xyz[0],
                                      bounding_box.max_xyz[1] - bounding_box.min_xyz[1]).norm();
      object_offsets[object] =
        pair<double, int>(object_configurations[room][object].center[0] - reference[0], object);
    }
    sort(object_offsets.begin(), object_offsets.end());

    const double object_size_sum = accumulate(object_sizes.begin(), object_sizes.end(), 0.0);
    const double object_scale = min(1.0, (x_range[1] - x_range[0]) / object_size_sum);
    
    double accumulated_position = - object_size_sum / 2.0;
    for (int i = 0; i < (int)object_offsets.size(); ++i) {
      const int object = object_offsets[i].second;
      const double position = accumulated_position + object_sizes[object] / 2.0;
      object_configurations[room][object].displacement =
        Vector3d(position * object_scale, 0.0, 0.0) -
        (object_configurations[room][object].center - reference);
      
      object_configurations[room][object].scale = kShrinkScale * object_scale;
      accumulated_position += object_sizes[object];
    }
  }
  */
}

Eigen::Vector3d ViewParameters::TransformRoom(const Vector3d& global,
                                             const int room,
                                             const double progress,
                                             const double animation,
                                             const Eigen::Vector3d& max_vertical_shift) const {
  double angle;
  if (animation < 0.25)
    angle = 2 * M_PI * animation;
  else if (0.25 <= animation && animation < 0.75)
    angle = 2 * M_PI * (0.5 - animation);
  else
    angle = 2 * M_PI * animation;
  
  Matrix3d rotation;
  rotation(0, 0) = cos(angle);
  rotation(0, 1) = -sin(angle);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(angle);
  rotation(1, 1) = cos(angle);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;

  const double scale = (1.0 - progress) * 1.0 + progress * room_configurations[room].scale;

  const double room_progress = min(2.0 * progress, 1.0);
  
  const Vector3d global_displacement =
    room_progress * (LocalToGlobalNormal(room_configurations[room].displacement) +
                     max_vertical_shift);

  Vector3d local = GlobalToLocal(global);

  local = room_configurations[room].center +
    scale * rotation * (local - room_configurations[room].center);

  return global_displacement + LocalToGlobal(local);
}

Eigen::Vector3d ViewParameters::TransformObject(const Vector3d& global,
                                               const int room,
                                               const int object,
                                               const double progress,
                                               const double animation,
                                               const Eigen::Vector3d& room_max_vertical_shift,
                                               const Eigen::Vector3d& object_max_vertical_shift) const {
  double angle;
  if (animation < 0.25)
    angle = 2 * M_PI * animation;
  else if (0.25 <= animation && animation < 0.75)
    angle = 2 * M_PI * (0.5 - animation);
  else
    angle = 2 * M_PI * animation;

  Matrix3d rotation;
  rotation(0, 0) = cos(angle);
  rotation(0, 1) = -sin(angle);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(angle);
  rotation(1, 1) = cos(angle);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;

  const Vector3d global_as_room = TransformRoom(global, room, progress, animation, room_max_vertical_shift);
  
  if (progress < 0.5) {
    return global_as_room;
  } else {
    const Vector3d row_shift = (object_max_vertical_shift - room_max_vertical_shift) / 3.0;
    const double scale = object_configurations[room][object].scale;
    const Vector3d global_displacement =
    (LocalToGlobalNormal(room_configurations[room].displacement +
                         object_configurations[room][object].displacement) +
     object_max_vertical_shift + object_configurations[room][object].row * row_shift);
    Vector3d local = GlobalToLocal(global);
    local = object_configurations[room][object].center +
      scale * rotation * (local - object_configurations[room][object].center);

    const Vector3d final_position = global_displacement + LocalToGlobal(local);
    const double progress2 = 2.0 * (progress - 0.5);
    return progress2 * final_position + (1.0 - progress2) * global_as_room;
  }
  
    /*
  const double scale = (1.0 - progress) * 1.0 + progress * object_configurations[room][object].scale;

  const double room_progress = min(2.0 * progress, 1.0);
  const double object_progress = 2.0 * max(0.0, (progress - 0.5));
  
  const Vector3d global_displacement =
    (LocalToGlobalNormal(room_progress * room_configurations[room].displacement +
                         object_progress * object_configurations[room][object].displacement) +
     progress * max_vertical_shift);
  //    progress *
  //    (LocalToGlobalNormal(room_configurations[room].displacement + object_configurations[room][object].displacement + Vector3d(0, 0, max_vertical_shift)));
    
  Vector3d local = GlobalToLocal(global);
  local = object_configurations[room][object].center +
    scale * rotation * (local - object_configurations[room][object].center);

  return global_displacement + LocalToGlobal(local);
  */
}


Eigen::Vector3d ViewParameters::TransformFloorplan(const Vector3d& global,
                                                  const double air_to_tree_progress,
                                                  const double animation,
                                                  const Eigen::Vector3d& max_vertical_shift,
                                                  const double max_shrink_scale) const {
  const double shrink_scale =
    (1.0 - air_to_tree_progress) + air_to_tree_progress * max_shrink_scale;
  const Eigen::Vector3d vertical_shift = air_to_tree_progress * max_vertical_shift;

  return center + (global - center) * shrink_scale + vertical_shift;
}

const Eigen::Vector3d ViewParameters::GetObjectCenter(const int room, const int object) const {
  return LocalToGlobal(object_configurations[room][object].center);
}
  
}  // namespace structured_indoor_modeling
