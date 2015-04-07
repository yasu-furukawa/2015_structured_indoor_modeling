#include <limits>
#include <numeric>
#include "view_parameters.h"
#include "configuration.h"
#include "../base/panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int ViewParameters::kMaxNumRows = 3;
  
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

void ViewParameters::Init(const double aspect_ratio_tmp) {
  aspect_ratio = aspect_ratio_tmp;
  
  room_configurations.resize(floorplan.GetNumRooms());
  floor_configurations.resize(floorplan.GetNumRooms());
  wall_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    wall_configurations[room].resize(floorplan.GetNumWalls(room));
  object_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    if (object_renderer.GetNumObjects(room) != 0)
      object_configurations[room].resize(object_renderer.GetNumObjects(room));
  }

  InitAxes();
  InitCenter();
  InitBoundingBoxes();
  InitAirFloorplanViewpoints();
  InitTreeConfigurationCenter();
  SetPolygonScale();
  SetRoomDisplacements();
  SetObjectDisplacements();
}


void ViewParameters::InitAxes() {
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

void ViewParameters::InitAirFloorplanViewpoints() {
  // diameter must be visible in the given field-of-view along air_angle.
  const double diameter0 = bounding_box.max_xyz[0] - bounding_box.min_xyz[0];
  const double diameter1 = bounding_box.max_xyz[1] - bounding_box.min_xyz[1];
  diameter = max(diameter0, diameter1 * aspect_ratio);
  
  air_height = diameter / 2.0 / tan(air_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(air_angle);
  air_height *= 1.2;
  
  floorplan_height = diameter / 2.0 / tan(floorplan_field_of_view_degrees / 2.0 * M_PI / 180.0) * sin(floorplan_angle);
  floorplan_height *= 1.2;
  
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

  const Vector3d center_local = floorplan.GetFloorplanToGlobal().transpose() * center;
  
  best_ground_center =
    floorplan.GetFloorplanToGlobal() *
    Vector3d(center_local[0], center_local[1], (average_floor_height + average_ceiling_height) / 2.0);

  for (int i = 0; i < 2; ++i) {
    if (i == 0)
      best_start_directions_for_air[i] = y_axis;
    else
      best_start_directions_for_air[i] = -y_axis;

    best_start_directions_for_floorplan[i] = best_start_directions_for_air[i];
        
    best_start_directions_for_air[i] += -tan(air_angle) * z_axis;
    best_start_directions_for_air[i] *= air_height / tan(air_angle);

    best_start_directions_for_floorplan[i] += -tan(floorplan_angle) * z_axis;
    best_start_directions_for_floorplan[i] *= floorplan_height / tan(floorplan_angle);
  }
}    
  
void ViewParameters::InitCenter() {
  center = Vector3d(0, 0, 0);
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

void ViewParameters::SetPolygonScale() {
  // Make the height occupy only 1/3 of the screen.
  const double target_height = diameter / aspect_ratio / 3.0;
  floorplan_scale = target_height / (bounding_box.max_xyz[1] - bounding_box.min_xyz[1]);
  
  vertical_floorplan = target_height;
}
  
void ViewParameters::SetRoomDisplacements() {
  // To allow some margin between items.
  vector<double> room_sizes(floorplan.GetNumRooms());
  vector<pair<double, int> > room_offsets(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const BoundingBox& bounding_box = room_configurations[room].bounding_box;
    room_sizes[room] = Vector2d(bounding_box.max_xyz[0] - bounding_box.min_xyz[0],
                                bounding_box.max_xyz[1] - bounding_box.min_xyz[1]).norm();
    room_offsets[room] = pair<double, int>(room_configurations[room].center[0], room);
  }
  sort(room_offsets.begin(), room_offsets.end());

  //----------------------------------------------------------------------
  // Room displacements.
  const double kExpandScale = 1.0; // 1.1;
  const double kShrinkScale = 0.95;
  const double room_size_sum = accumulate(room_sizes.begin(), room_sizes.end(), 0.0);

  indoor_polygon_scale = kExpandScale * diameter / room_size_sum;

  double accumulated_position = - room_size_sum / 2.0;

  for (int i = 0; i < (int)room_offsets.size(); ++i) {
    const int room = room_offsets[i].second;
    const double position = accumulated_position + room_sizes[room] / 2.0;
    room_configurations[room].displacement =
      Vector3d(position * indoor_polygon_scale, 0, 0) - room_configurations[room].center;
    
    room_configurations[room].scale = kShrinkScale * indoor_polygon_scale;
    accumulated_position += room_sizes[room];
  }
}

void ViewParameters::SetObjectDisplacements() {
  // const double kShrinkScale = 0.95;
  //----------------------------------------------------------------------
  // Object displacements.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    if (object_renderer.GetNumObjects(room) == 0)
      continue;
    
    vector<double> object_sizes(object_renderer.GetNumObjects(room));
    vector<pair<double, int> > object_offsets(object_renderer.GetNumObjects(room));
    const Eigen::Vector3d& reference = room_configurations[room].center;

    Vector3d diff = room_configurations[room].bounding_box.min_xyz -
      room_configurations[room].bounding_box.max_xyz;
    diff[2] = 0.0;
    const double radius = diff.norm();
    
    const Vector2d x_range(- room_configurations[room].scale * radius / 2,
                           room_configurations[room].scale * radius / 2);

    //(room_configurations[room].bounding_box.min_xyz[0] - reference[0]),
    //indoor_polygon_scale * (room_configurations[room].bounding_box.max_xyz[0] - reference[0]));

    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const BoundingBox& bounding_box = object_configurations[room][object].bounding_box;
      object_sizes[object] = Vector2d(bounding_box.max_xyz[0] - bounding_box.min_xyz[0],
                                      bounding_box.max_xyz[1] - bounding_box.min_xyz[1]).norm();

      object_offsets[object] =
        pair<double, int>(object_configurations[room][object].center[0] - reference[0], object);
    }
    sort(object_offsets.begin(), object_offsets.end());

    const double object_size_sum = accumulate(object_sizes.begin(), object_sizes.end(), 0.0);
    const int num_rows = min(kMaxNumRows,
                             static_cast<int>(ceil(object_size_sum / round(scaled_x_range[1] - scaled_x_range[0]))));

    const int num_items_per_row =
      static_cast<int>(ceil(object_renderer.GetNumObjects(room) / (double)num_rows));
    for (int row = 0; row < num_rows; ++row) {
      const int start_index = num_items_per_row * row;
      const int end_index = min(object_renderer.GetNumObjects(room), num_items_per_row * (row + 1));

      double object_size_sum_per_row = 0.0;
      for (int i = start_index; i < end_index; ++i) {
        object_size_sum_per_row += object_sizes[object_offsets[i].second];
      }

      const int num_objects_per_row = end_index - start_index;
      const double object_scale = min(1.0, (x_range[1] - x_range[0]) / object_size_sum_per_row);

      const double scaled_total_size = object_scale * object_size_sum_per_row;
      const double scaled_margin = ((scaled_x_range[1] - scaled_x_range[0]) - scaled_total_size) / num_objects_per_row;

      double accumulated_position = - (x_range[1] - x_range[0]) / 2.0 + margin / 2.0;
      for (int i = start_index; i < end_index; ++i) {
        const int object = object_offsets[i].second;
        const double position = accumulated_position + object_scale * object_sizes[object] / 2.0;
        object_configurations[room][object].displacement =
          Vector3d(position, 0.0, 0.0) -
          (object_configurations[room][object].center - reference);
        
        object_configurations[room][object].scale = object_scale;
        object_configurations[room][object].row = row;
        accumulated_position += object_scale * object_sizes[object] + scaled_margin;
      }
      
      /*
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
      */
    }
  }  
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
                                                const Eigen::Vector3d& vertical_object_top,
                                                const Eigen::Vector3d& vertical_object_bottom) const {
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
    const Vector3d row_shift = (vertical_object_bottom - vertical_object_top) / kMaxNumRows;
    const double scale = object_configurations[room][object].scale;
    const Vector3d global_displacement =
      (LocalToGlobalNormal(room_configurations[room].displacement +
                           object_configurations[room][object].displacement) +
       vertical_object_top + (0.5 + object_configurations[room][object].row) * row_shift);
    Vector3d local = GlobalToLocal(global);
    local = object_configurations[room][object].center +
      scale * rotation * (local - object_configurations[room][object].center);

    const Vector3d final_position = global_displacement + LocalToGlobal(local);
    const double progress2 = 2.0 * (progress - 0.5);
    return progress2 * final_position + (1.0 - progress2) * global_as_room;
  }
}

Eigen::Vector3d ViewParameters::TransformFloorplan(const Vector3d& global,
                                                  const double air_to_tree_progress,
                                                   const double /* animation */,
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

void ViewParameters::SetOffsetDirection(const Eigen::Vector3d& view_direction,
                                        Eigen::Vector3d* offset_direction) const {
  const Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  const Vector3d vertical = floorplan_to_global * Vector3d(0, 0, -1);
  const Vector3d orthogonal = vertical.cross(view_direction);
  *offset_direction = (orthogonal.cross(view_direction)).normalized();
}
  
void ViewParameters::SetBoundaries(const Eigen::Vector3d& offset_direction,
                                   std::vector<std::vector<Eigen::Vector3d> >* boundaries) const {
  boundaries->clear();
  boundaries->resize(floorplan.GetNumRooms());

  const Vector3d top_offset    = GetVerticalTopBoundary() * offset_direction;
  const Vector3d bottom_offset = GetVerticalBottomBoundary() * offset_direction;
  
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    /*
    double min_x = room_configurations[room].bounding_box.min_xyz[0];
    double max_x = room_configurations[room].bounding_box.max_xyz[0];
    const double center_x = room_configurations[room].center[0];
    const double kEnlarge = 1.1;
    min_x = center_x + (min_x - center_x) * room_configurations[room].scale * kEnlarge;
    max_x = center_x + (max_x - center_x) * room_configurations[room].scale * kEnlarge;

    //?????
    Vector3d left(min_x,
                  room_configurations[room].center[1],
                  //0);
                  room_configurations[room].center[2]);
    Vector3d right(max_x,
                   room_configurations[room].center[1],
                   //                   0);
                   room_configurations[room].center[2]);
    */
    Vector3d diff = room_configurations[room].bounding_box.min_xyz -
      room_configurations[room].bounding_box.max_xyz;
    diff[2] = 0.0;
    const double radius = diff.norm();

    const double kShrink = 1.0; // 0.95;
    const double min_x = room_configurations[room].center[0] - kShrink * room_configurations[room].scale * radius / 2.0;
    const double max_x = room_configurations[room].center[0] + kShrink * room_configurations[room].scale * radius / 2.0;

    Vector3d left(min_x,
                  room_configurations[room].center[1],
                  room_configurations[room].center[2]);
    Vector3d right(max_x,
                   room_configurations[room].center[1],
                   room_configurations[room].center[2]);
                   
    left  += room_configurations[room].displacement;
    right += room_configurations[room].displacement;

    boundaries->at(room).push_back(LocalToGlobal(left)  + top_offset);
    boundaries->at(room).push_back(LocalToGlobal(right) + top_offset);
    boundaries->at(room).push_back(LocalToGlobal(right) + bottom_offset);
    boundaries->at(room).push_back(LocalToGlobal(left)  + bottom_offset);
  }
}

void ViewParameters::SetLines(const Eigen::Vector3d& offset_direction,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* top_lines,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* bottom_lines) const {
  const Vector3d top_offset = GetVerticalTopBoundary() * offset_direction;

  const double kProgressIrrelevant = 1.0;
  const double kAnimationIrrelevant = 0.0;
  const Vector3d kNoOffset(0.0, 0.0, 0.0);
  
  top_lines->clear();
  bottom_lines->clear();
  top_lines->resize(floorplan.GetNumRooms());
  bottom_lines->resize(floorplan.GetNumRooms());
  
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const Vector2d local = floorplan.GetRoomCenterLocal(room);
    const Vector3d floor_local(local[0], local[1], floorplan.GetFloorHeight(room));
    const Vector3d ceiling_local(local[0], local[1], floorplan.GetCeilingHeight(room));
    const Vector3d floor_global = floorplan.GetFloorplanToGlobal() * floor_local;
    const Vector3d ceiling_global = floorplan.GetFloorplanToGlobal() * ceiling_local;
    const Vector3d room_global = // (floor_global + ceiling_global) / 2.0;
      LocalToGlobal(Vector3d(room_configurations[room].center[0],
                             room_configurations[room].center[1],
                             room_configurations[room].center[2]));
                    //                             0.0));

    top_lines->at(room) = make_pair(TransformFloorplan(floor_global,
                                                       kProgressIrrelevant,
                                                       kAnimationIrrelevant,
                                                       GetVerticalFloorplan() * offset_direction,
                                                       GetFloorplanScale()),
                                    TransformRoom(ceiling_global,
                                                  room,
                                                  kProgressIrrelevant,
                                                  kAnimationIrrelevant,
                                                  kNoOffset));
    
    bottom_lines->at(room) = make_pair(TransformRoom(floor_global,
                                                     room,
                                                     kProgressIrrelevant,
                                                     kAnimationIrrelevant,
                                                     kNoOffset),
                                       TransformRoom(room_global,
                                                     room,
                                                     kProgressIrrelevant,
                                                     kAnimationIrrelevant,
                                                     top_offset));
  }
}

double ViewParameters::SetAnimationAlpha(const double animation) {
  const double kMargin = 0.05;
  const double pivots[4] = { 1.0 / 8, 3.0 / 8, 5.0 / 8, 7.0 / 8};
  double diff = 1.0;
  for (int i = 0; i < 4; ++i)
    diff = min(diff, fabs(animation - pivots[i]));
  return max(0.0, min(1.0, 2.0 * (kMargin - diff) / kMargin));
}
  
}  // namespace structured_indoor_modeling
