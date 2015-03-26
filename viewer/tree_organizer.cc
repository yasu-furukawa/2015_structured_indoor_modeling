#include <limits>
#include "tree_organizer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

TreeOrganizer::TreeOrganizer(const Floorplan& floorplan,
                             const IndoorPolygon& indoor_polygon,
                             const ObjectRenderer& object_renderer) :
  floorplan(floorplan), indoor_polygon(indoor_polygon), object_renderer(object_renderer) {
}

void TreeOrganizer::Init(const Eigen::Vector3d& tree_layout_direction,
                         const Eigen::Vector3d& tree_layout_orthogonal_direction) {
  InitFloorplanDeformation();
  InitIndoorPolygonDeformation();
  InitObjectDeformation();

  // Compute displacements for floorplan_deformation.
  ComputeDisplacementsFloorplan(tree_layout_direction, tree_layout_orthgonal_direction);

  indoor_polygon_deformation.displacements = floorplan_deformation.displacements;

  // Compute displacements for objects.
}

const FloorplanDeformation& TreeOrganizer::GetFloorplanDeformation() const {
  return floorplan_deformation;
}
  
const FloorplanDeformation& TreeOrganizer::GetIndoorPolygonDeformation() const {
  return indoor_polygon_deformation;
}

const ObjectDeformation& TreeOrganizer::GetObjectDeformation() const {
  return object_deformation;
}
  
void TreeOrganizer::InitFloorplanDeformation() {
  {
    floorplan_deformation.room_bounding_boxes.clear();
    floorplan_deformation.room_bounding_boxes.resize(floorplan.GetNumRooms());
    
    floorplan_deformation.floor_bounding_boxes.clear();
    floorplan_deformation.floor_bounding_boxes.resize(floorplan.GetNumRooms());
    
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      // Compute the bounding of the room.
      Vector3d min_xyz, max_xyz;
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        if (v == 0) {
          min_xyz = max_xyz = floorplan.GetFloorVertexGlobal(room, v);
        } else {
          const Eigen::Vector3d& floor_vertex = floorplan.GetFloorVertexGlobal(room, v);
          for (int a = 0; a < 3; ++a) {
            min_xyz[a] = min(min_xyz[a], floor_vertex[a]);
            max_xyz[a] = max(max_xyz[a], floor_vertex[a]);
          }
        }
      }
      
      floorplan_deformation.floor_bounding_boxes[room].min_xyz = min_xyz;
      floorplan_deformation.floor_bounding_boxes[room].max_xyz = max_xyz;
      
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        const Eigen::Vector3d& ceiling_vertex = floorplan.GetCeilingVertexGlobal(room, v);
        for (int a = 0; a < 3; ++a) {
          min_xyz[a] = min(min_xyz[a], ceiling_vertex[a]);
          max_xyz[a] = max(max_xyz[a], ceiling_vertex[a]);
        }
      }
      floorplan_deformation.room_bounding_boxes[room].min_xyz = min_xyz;
      floorplan_deformation.room_bounding_boxes[room].max_xyz = max_xyz;
    }
  }    

  {
    floorplan_deformation.wall_bounding_boxes.clear();
    floorplan_deformation.wall_bounding_boxes.resize(floorplan.GetNumRooms());

    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      floorplan_deformation.wall_bounding_boxes[room].resize(floorplan.GetNumRoomVertices(room));
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        const int next_v = (v + 1) % floorplan.GetNumRoomVertices(room);

        for (int a = 0; a < 3; ++a) {
          floorplan_deformation.wall_bounding_boxes[room][v].min_xyz[a] = 
            min(min(floorplan.GetFloorVertexGlobal(room, v)[a],
                    floorplan.GetFloorVertexGlobal(room, next_v)[a]),
                min(floorplan.GetCeilingVertexGlobal(room, v)[a],
                    floorplan.GetCeilingVertexGlobal(room, next_v)[a]));
          floorplan_deformation.wall_bounding_boxes[room][v].max_xyz[a] = 
            max(max(floorplan.GetFloorVertexGlobal(room, v)[a],
                    floorplan.GetFloorVertexGlobal(room, next_v)[a]),
                max(floorplan.GetCeilingVertexGlobal(room, v)[a],
                    floorplan.GetCeilingVertexGlobal(room, next_v)[a]));
        }
      }
    }
  }
}
  
void TreeOrganizer::InitIndoorPolygonDeformation() {
  indoor_polygon_deformation.room_bounding_boxes.clear();
  indoor_polygon_deformation.room_bounding_boxes.resize(floorplan.GetNumRooms());
  indoor_polygon_deformation.floor_bounding_boxes.clear();
  indoor_polygon_deformation.floor_bounding_boxes.resize(floorplan.GetNumRooms());

  indoor_polygon_deformation.wall_bounding_boxes.clear();
  indoor_polygon_deformation.wall_bounding_boxes.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room)
    indoor_polygon_deformation.wall_bounding_boxes[room].resize(floorplan.GetNumWalls(room));
  
  const Eigen::Vector3d kMinInitial(numeric_limits<double>::max(),
                                    numeric_limits<double>::max(),
                                    numeric_limits<double>::max());
  const Eigen::Vector3d kMaxInitial(- numeric_limits<double>::max(),
                                    - numeric_limits<double>::max(),
                                    - numeric_limits<double>::max());
                             
  vector<Eigen::Vector3d> room_min_xyzs(floorplan.GetNumRooms(), kMinInitial);
  vector<Eigen::Vector3d> room_max_xyzs(floorplan.GetNumRooms(), kMaxInitial);

  vector<Eigen::Vector3d> floor_min_xyzs(floorplan.GetNumRooms(), kMinInitial);
  vector<Eigen::Vector3d> floor_max_xyzs(floorplan.GetNumRooms(), kMaxInitial);
  
  vector<vector<Eigen::Vector3d> > wall_min_xyzs(floorplan.GetNumRooms());
  vector<vector<Eigen::Vector3d> > wall_max_xyzs(floorplan.GetNumRooms());
  
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    wall_min_xyzs[room].resize(floorplan.GetNumWalls(room), kMinInitial);
    wall_max_xyzs[room].resize(floorplan.GetNumWalls(room), kMaxInitial);
  }

  for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
    const Segment& segment = indoor_polygon.GetSegment(s);
    if (segment.type == Segment::WALL) {
      const int room = segment.wall_info[0];
      const int wall = segment.wall_info[1];

      for (const auto& vertex : segment.vertices) {
        const Vector3d& global = indoor_polygon.ManhattanToGlobal(vertex);

        for (int a = 0; a < 3; ++a) {
          room_min_xyzs[room][a] = min(room_min_xyzs[room][a], global[a]);
          room_max_xyzs[room][a] = max(room_max_xyzs[room][a], global[a]);

          wall_min_xyzs[room][wall][a] = min(wall_min_xyzs[room][wall][a], global[a]);
          wall_max_xyzs[room][wall][a] = max(wall_max_xyzs[room][wall][a], global[a]);
        }
      }
    } else if (segment.type == Segment::FLOOR) {
      const int room = segment.floor_info;

      for (const auto& vertex : segment.vertices) {
        const Vector3d& global = indoor_polygon.ManhattanToGlobal(vertex);
        
        for (int a = 0; a < 3; ++a) {
          floor_min_xyzs[room][a] = min(floor_min_xyzs[room][a], global[a]);
          floor_max_xyzs[room][a] = max(floor_max_xyzs[room][a], global[a]);
        }
      }
    }
  }

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    indoor_polygon_deformation.room_bounding_boxes[room].min_xyz = room_min_xyzs[room];
    indoor_polygon_deformation.room_bounding_boxes[room].max_xyz = room_max_xyzs[room];
    
    indoor_polygon_deformation.floor_bounding_boxes[room].min_xyz = floor_min_xyzs[room];
    indoor_polygon_deformation.floor_bounding_boxes[room].max_xyz = floor_max_xyzs[room];
    
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      indoor_polygon_deformation.wall_bounding_boxes[room][wall].min_xyz =
        wall_min_xyzs[room][wall];
      indoor_polygon_deformation.wall_bounding_boxes[room][wall].max_xyz =
        wall_max_xyzs[room][wall];
    }
  }
}
  
void TreeOrganizer::InitObjectDeformation() {
  object_deformation.bounding_boxes.clear();
  object_deformation.bounding_boxes.resize(object_renderer.GetNumRooms());

  for (int room = 0; room < object_renderer.GetNumRooms(); ++room) {
    object_deformation.bounding_boxes[room].resize(object_renderer.GetNumObjects(room));
    
    for (int object = 0; object < object_renderer.GetNumObjects(room); ++object) {
      const vector<float>& points = object_renderer.GetObject(room, object);
      Vector3d min_xyz(numeric_limits<double>::max(),
                       numeric_limits<double>::max(),
                       numeric_limits<double>::max());
      Vector3d max_xyz(- numeric_limits<double>::max(),
                       - numeric_limits<double>::max(),
                       - numeric_limits<double>::max());

      for (int v = 0; v < (int)points.size(); v += 3) {
        for (int a = 0; a < 3; ++a) {
          min_xyz[a] = min(min_xyz[a], static_cast<double>(points[v + a]));
          max_xyz[a] = max(max_xyz[a], static_cast<double>(points[v + a]));
        }
      }

      object_deformation.bounding_boxes[room][object].min_xyz = min_xyz;
      object_deformation.bounding_boxes[room][object].max_xyz = max_xyz;
    }
  }
}

}  // namespace structured_indoor_modeling
