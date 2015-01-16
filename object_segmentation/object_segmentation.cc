#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../base/floorplan.h"
#include "../base/point_cloud.h"
#include "object_segmentation.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

bool IsOnFloor(const double floor_height, const double margin, const Point& point) {
  return fabs(point.position[2] - floor_height) <= margin;
}
  
bool IsOnWall(const std::vector<cv::Point>& contour,
              const double wall_margin,
              const Point& point) {
  const cv::Point local(point.position[0], point.position[1]);
  return fabs(cv::pointPolygonTest(contour, local, true)) <= wall_margin;
}

bool IsOnCeiling(const double ceiling_height, const double margin, const Point& point) {
  return fabs(point.position[2] - ceiling_height) <= margin;
}  

}  // namespace

void SetRoomOccupancy(const Floorplan& floorplan,
                      std::vector<int>* room_occupancy) {
  const Vector3i grid_size = floorplan.GetGridSize();
  room_occupancy->clear();
  const int kBackground = -1;
  room_occupancy->resize(grid_size[0] * grid_size[1], kBackground);

  // Make room contour polygons for each room.
  vector<vector<cv::Point> > room_contours(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const Vector2d local = floorplan.GetRoomVertexLocal(room, vertex);
      room_contours[room].push_back(cv::Point(local[0], local[1]));
    }
  }

  const double kDistanceThreshold = 0;//floorplan.GetGridUnit() * 2.0;
  int index = 0;
  for (int y = 0; y < grid_size[1]; ++y) {
    for (int x = 0; x < grid_size[0]; ++x, ++index) {
      const Vector2d local = floorplan.GridToLocal(Vector2d(x, y));
      const cv::Point point(local[0], local[1]);

      // Find the room with the closest distance
      const int kInvalid = -1;
      int best_room = kInvalid;
      double best_distance = 0.0;

      for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
        const double distance = cv::pointPolygonTest(room_contours[room], point, true);
        if (best_room == kInvalid || distance > best_distance) {
          best_room = room;
          best_distance = distance;

          // Inside.
          if (best_distance > 0.0)
            break;
        }
      }
      
      if (best_distance > - kDistanceThreshold) {
        room_occupancy->at(index) = best_room;
      }
    }
  }
}

void CollectPointsInRoom(const std::vector<PointCloud>& point_clouds,
                         const Floorplan& floorplan,
                         const std::vector<int>& room_occupancy,
                         const int room,
                         std::vector<Point>* points) {
  for (const auto& point_cloud : point_clouds) {
    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Vector3d& local = point_cloud.GetPoint(p).position;
      const Vector2i grid_int = floorplan.LocalToGridInt(Vector2d(local[0], local[1]));
      const int index = grid_int[1] * floorplan.GetGridSize()[0] + grid_int[0];
      if (room_occupancy[index] == room)
        points->push_back(point_cloud.GetPoint(p));
    }
  }
}

void IdentifyFloorWallCeiling(const std::vector<Point>& points,
                              const Floorplan& floorplan,
                              const std::vector<int>& room_occupancy,
                              const int room,
                              std::vector<RoomSegment>* segments) {
  const double kFloorMarginRatio   = 0.03;
  const double kCeilingMarginRatio = 0.1;
  const double kWallMarginRatio    = 0.03;
  const double room_height    = floorplan.GetCeilingHeight(room) - floorplan.GetFloorHeight(room);
  const double floor_margin   = room_height * kFloorMarginRatio;
  const double wall_margin    = room_height * kWallMarginRatio;
  const double ceiling_margin = room_height * kCeilingMarginRatio;

  vector<cv::Point> contour;
  for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
    const Vector2d local = floorplan.GetRoomVertexLocal(room, vertex);
    contour.push_back(cv::Point(local[0], local[1]));
  }
  
  segments->clear();
  segments->resize((int)points.size(), kInitial);
  // Floor check.
  for (int p = 0; p < (int)points.size(); ++p) {
    if (IsOnFloor(floorplan.GetFloorHeight(room), floor_margin, points[p]))
      segments->at(p) = kFloor;
    else if (IsOnCeiling(floorplan.GetCeilingHeight(room), ceiling_margin, points[p]))
      segments->at(p) = kCeiling;
    else if (IsOnWall(contour, wall_margin, points[p]))
      segments->at(p) = kWall;
  }


}  
  
}  // namespace structured_indoor_modeling
