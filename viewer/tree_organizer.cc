#include <limits>
#include <numeric>
#include "tree_organizer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

TreeOrganizer::TreeOrganizer(const Floorplan& floorplan,
                             const IndoorPolygon& indoor_polygon,
                             const ObjectRenderer& object_renderer) :
  floorplan(floorplan), indoor_polygon(indoor_polygon), object_renderer(object_renderer) {
}

void TreeOrganizer::Init(const Eigen::Vector3d& x_axis_tmp,
                         const Eigen::Vector3d& y_axis_tmp,
                         const Eigen::Vector3d& z_axis_tmp) {
  room_configurations.resize(floorplan.GetNumRooms());
  floor_configurations.resize(floorplan.GetNumRooms());
  wall_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    wall_configurations[room].resize(floorplan.GetNumWalls(room));
  object_configurations.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    object_configurations[room].resize(object_renderer.GetNumObjects(room));

  x_axis = x_axis_tmp;
  y_axis = y_axis_tmp;
  z_axis = z_axis_tmp;

  InitCenter();
  InitBoundingBoxes();
  InitTreeConfigurationCenter();
  SetDisplacements();
}

void TreeOrganizer::InitCenter() {
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

void TreeOrganizer::InitBoundingBoxes() {
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

void TreeOrganizer::InitTreeConfigurationCenter() {
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

void TreeOrganizer::SetDisplacements() {
  vector<double> room_sizes(floorplan.GetNumRooms());
  vector<pair<double, int> > room_offsets(floorplan.GetNumRooms());
  Vector2d x_range(numeric_limits<double>::max(), - numeric_limits<double>::max());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const BoundingBox& bounding_box = room_configurations[room].bounding_box;
    room_sizes[room] = max(fabs(bounding_box.max_xyz[0] - bounding_box.min_xyz[0]),
                           fabs(bounding_box.max_xyz[1] - bounding_box.min_xyz[1]));
    room_offsets[room] = pair<double, int>(room_configurations[room].center[0], room);

    x_range[0] = min(x_range[0], bounding_box.min_xyz[0]);
    x_range[1] = max(x_range[1], bounding_box.max_xyz[0]);
  }
  sort(room_offsets.begin(), room_offsets.end());

  //----------------------------------------------------------------------
  // Room displacements.
  
  const double room_size_sum = accumulate(room_sizes.begin(), room_sizes.end(), 0.0);
  const double scale = (x_range[1] - x_range[0]) / room_size_sum;

  double accumulated_position = x_range[0];
  for (int i = 0; i < (int)room_offsets.size(); ++i) {
    const int room = room_offsets[i].second;
    const double position = accumulated_position + room_sizes[room] / 2.0;
    room_configurations[room].displacement =
      Vector3d(position * scale, 0.0, 0.0) - room_configurations[room].center;
    
    room_configurations[room].scale = scale;
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
    const Vector2d x_range(room_configurations[room].bounding_box.min_xyz[0] - reference[0],
                           room_configurations[room].bounding_box.max_xyz[0] - reference[0]);

    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const BoundingBox& bounding_box = object_configurations[room][object].bounding_box;
      object_sizes[object] = max(fabs(bounding_box.max_xyz[0] - bounding_box.min_xyz[0]),
                                 fabs(bounding_box.max_xyz[1] - bounding_box.min_xyz[1]));
      object_offsets[object] =
        pair<double, int>(object_configurations[room][object].center[0] - reference[0], object);
    }
    sort(object_offsets.begin(), object_offsets.end());

    const double object_size_num = accumulate(object_sizes.begin(), object_sizes.end(), 0.0);
    const double object_scale = (x_range[1] - x_range[0]) / object_size_num;
    
    double accumulated_position = x_range[0];
    for (int i = 0; i < (int)object_offsets.size(); ++i) {
      const int object = object_offsets[i].second;
      const double position = accumulated_position + object_sizes[object] / 2.0;
      object_configurations[room][object].displacement =
        Vector3d(position * object_scale, 0.0, 0.0) -
        (object_configurations[room][object].center - reference);
      
      object_configurations[room][object].scale = object_scale;
      accumulated_position += object_sizes[object];
    }
  }
}

Eigen::Vector3d TreeOrganizer::TransformRoom(const Vector3d& global,
                                             const int room,
                                             const double progress,
                                             const double animation,
                                             const double max_vertical_shift) const {
  Matrix3d rotation;
  rotation(0, 0) = cos(animation * 2 * M_PI);
  rotation(0, 1) = -sin(animation * 2 * M_PI);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(animation * 2 * M_PI);
  rotation(1, 1) = cos(animation * 2 * M_PI);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;
    
  const Vector3d global_displacement = progress * (LocalToGlobal(room_configurations[room].displacement + Vector3d(0, 0, max_vertical_shift)));

  Vector3d local = GlobalToLocal(global);
  local = local + room_configurations[room].scale * rotation * (local - room_configurations[room].center);

  return global_displacement + LocalToGlobal(local);
}

Eigen::Vector3d TreeOrganizer::TransformObject(const Vector3d& global,
                                               const int room,
                                               const int object,
                                               const double progress,
                                               const double animation,
                                               const double max_vertical_shift) const {
  Matrix3d rotation;
  rotation(0, 0) = cos(animation * 2 * M_PI);
  rotation(0, 1) = -sin(animation * 2 * M_PI);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(animation * 2 * M_PI);
  rotation(1, 1) = cos(animation * 2 * M_PI);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;
    
  const Vector3d global_displacement =
    progress *
    (LocalToGlobal(room_configurations[room].displacement + object_configurations[room][object].displacement + Vector3d(0, 0, max_vertical_shift)));

  Vector3d local = GlobalToLocal(global);
  local = local + object_configurations[room][object].scale * rotation * (local - object_configurations[room][object].center);

  return global_displacement + LocalToGlobal(local);
}
}  // namespace structured_indoor_modeling
