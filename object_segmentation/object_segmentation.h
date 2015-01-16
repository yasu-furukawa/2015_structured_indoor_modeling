#ifndef OBJECT_SEGMENTATION_H_
#define OBJECT_SEGMENTATION_H_

#include <vector>

namespace structured_indoor_modeling {

// room segment id definitions.
const int kInitial = -1;
const int kFloor = -2;
const int kWall = -3;
const int kCeiling = -4;
  
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
                              std::vector<int>* segments);                          

void Subsample(const double ratio, std::vector<Point>* points);
 
void FilterNoisyPoints(std::vector<Point>* points);
 
void SegmentObjects(const std::vector<Point>& points,
                    const double centroid_subsampling_ratio,
                    std::vector<int>* segments);

void SetNeighbors(const std::vector<Point>& points,
                  const int num_neighbors,
                  std::vector<std::vector<int> >* neighbors); 
 
}  // namespace structured_indoor_modeling

#endif  // OBJECT_SEGMENTATION_H_
