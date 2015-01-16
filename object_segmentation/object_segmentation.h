#ifndef OBJECT_SEGMENTATION_H_
#define OBJECT_SEGMENTATION_H_

#include <vector>

namespace structured_indoor_modeling {

// room segment id definitions.
enum RoomSegment {
  kInitial = -1,
  kFloor = -2,
  kWall = -3,
  kCeiling = -4
};
  
class Floorplan;
class PointCloud;
struct Point;

void SetRoomOccupancy(const Floorplan& floorplan,
                      std::vector<int>* room_occupancy);

void CollectPointsInRoom(const std::vector<PointCloud>& point_clouds,
                         const Floorplan& floorplan,
                         const std::vector<int>& room_occupancy,
                         const int room,
                         std::vector<Point>* points);                          

void IdentifyFloorWallCeiling(const std::vector<Point>& points,
                              const Floorplan& floorplan,
                              const std::vector<int>& room_occupancy,
                              const int room,
                              std::vector<RoomSegment>* segments);                          
 
}  // namespace structured_indoor_modeling

#endif  // OBJECT_SEGMENTATION_H_
