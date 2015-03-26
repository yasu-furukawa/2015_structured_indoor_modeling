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

void TreeOrganizer::Init() {
  InitFloorplanCenters();
  InitIndoorPolygonCenters();
  InitObjectCenters();

}

void TreeOrganizer::InitFloorplanCenters() {
  {
    floorplan_centers.room_centers.clear();
    floorplan_centers.room_centers.resize(floorplan.GetNumRooms());
    
    floorplan_centers.floor_centers.clear();
    floorplan_centers.floor_centers.resize(floorplan.GetNumRooms());
    
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
      
      floorplan_centers.floor_centers[room] = (min_xyz + max_xyz) / 2.0;
      
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        const Eigen::Vector3d& ceiling_vertex = floorplan.GetCeilingVertexGlobal(room, v);
        for (int a = 0; a < 3; ++a) {
          min_xyz[a] = min(min_xyz[a], ceiling_vertex[a]);
          max_xyz[a] = max(max_xyz[a], ceiling_vertex[a]);
        }
      }
      floorplan_centers.room_centers[room] = (min_xyz + max_xyz) / 2.0;
    }
  }    

  {
    floorplan_centers.wall_centers.clear();
    floorplan_centers.wall_centers.resize(floorplan.GetNumRooms());

    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      floorplan_centers.wall_centers[room].resize(floorplan.GetNumRoomVertices(room));
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        const int next_v = (v + 1) % floorplan.GetNumRoomVertices(room);
        
        floorplan_centers.wall_centers[room][v] = 
          (floorplan.GetFloorVertexGlobal(room, v) +
           floorplan.GetFloorVertexGlobal(room, next_v) +
           floorplan.GetCeilingVertexGlobal(room, v) +
           floorplan.GetCeilingVertexGlobal(room, next_v)) / 4.0;
      }
    }
  }
}
  
void TreeOrganizer::InitIndoorPolygonCenters() {
  indoor_polygon_centers.room_centers.clear();
  indoor_polygon_centers.room_centers.resize(floorplan.GetNumRooms());

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
        for (int a = 0; a < 3; ++a) {
          room_min_xyzs[room][a] = min(room_min_xyzs[room][a], vertex[a]);
          room_max_xyzs[room][a] = max(room_max_xyzs[room][a], vertex[a]);

          wall_min_xyzs[room][wall][a] = min(wall_min_xyzs[room][wall][a], vertex[a]);
          wall_max_xyzs[room][wall][a] = max(wall_max_xyzs[room][wall][a], vertex[a]);
        }
      }
    } else if (segment.type == Segment::FLOOR) {
      const int room = segment.floor_info;

      for (const auto& vertex : segment.vertices) {
        for (int a = 0; a < 3; ++a) {
          floor_min_xyzs[room][a] = min(floor_min_xyzs[room][a], vertex[a]);
          floor_max_xyzs[room][a] = max(floor_max_xyzs[room][a], vertex[a]);
        }
      }
    }
  }


  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    indoor_polygon_centers.room_centers[room] = (room_min_xyzs[room] + room_max_xyzs[room]) / 2.0;
    indoor_polygon_centers.floor_centers[room] = (floor_min_xyzs[room] + floor_max_xyzs[room]) / 2.0;

    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      indoor_polygon_centers.wall_centers[room][wall] =
        (wall_min_xyzs[room][wall] + wall_max_xyzs[room][wall]) / 2.0;
    }
  }
}
  
void TreeOrganizer::InitObjectCenters() {
  object_centers.centers.clear();
  object_centers.centers.resize(object_renderer.GetNumRooms());

  for (int room = 0; room < object_renderer.GetNumRooms(); ++room) {
    object_centers.centers[room].resize(object_renderer.GetNumObjects(room));
    
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

      object_centers.centers[room][object] = (min_xyz + max_xyz) / 2.0;
    }
  }
}

}  // namespace structured_indoor_modeling
