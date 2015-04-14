#ifndef OBJECT_SEGMENTATION_H_
#define OBJECT_SEGMENTATION_H_

#include <vector>

namespace structured_indoor_modeling {

// room segment id definitions.
const int kInitial = -1;
const int kFloor = -2;
const int kWall = -3;
const int kCeiling = -4;
const int kDetail = -5;
  
class Floorplan;
class IndoorPolygon;
class PointCloud;
struct Point;

void SaveData(const int id,
              const std::vector<Point>& points,
              const std::vector<int>& segments);

void LoadData(const int id,
              std::vector<Point>* points,
              std::vector<int>* segments);
 
void SetRoomOccupancy(const Floorplan& floorplan,
                      std::vector<int>* room_occupancy);

void SetDoorOccupancy(const Floorplan& floorplan,
                      std::vector<int>* room_occupancy_with_doors);
 
void CollectPointsInRoom(const std::vector<PointCloud>& point_clouds,
                         const Floorplan& floorplan,
                         const std::vector<int>& room_occupancy,
                         const int room,
                         std::vector<Point>* points);                          

void IdentifyFloorWallCeiling(const std::vector<Point>& points,
                              const Floorplan& floorplan,
                              const int room,
                              const double rescale_margin,
                              std::vector<int>* segments);                          

void IdentifyDetails(const std::vector<Point>& points,
                     const Floorplan& floorplan,
                     const IndoorPolygon& indoor_polygon,
                     const int room,
                     const double rescale_margin,
                     std::vector<int>* segments);                          
 
void Subsample(const double ratio, std::vector<Point>* points);
 
void FilterNoisyPoints(std::vector<Point>* points);
 
void SegmentObjects(const std::vector<Point>& points,
                    const double centroid_subsampling_ratio,
                    const int num_initial_clusters,
                    const std::vector<std::vector<int> >& neighbors,
                    std::vector<int>* segments);

void SmoothObjects(const std::vector<std::vector<int> >& neighbors,
                   std::vector<Point>* points);

void DensifyObjects(const std::vector<std::vector<int> >& neighbors,
                    std::vector<Point>* points,
                    std::vector<int>* segments);
 
void SetNeighbors(const std::vector<Point>& points,
                  const int num_neighbors,
                  std::vector<std::vector<int> >* neighbors);

void RemoveWindowAndMirror(const Floorplan& floorplan,
                           const std::vector<int>& room_occupancy_with_doors,
                           const Eigen::Vector3d& center,
                           PointCloud* point_cloud);

void WriteObjectPointsWithColor(const std::vector<Point>& points,
                                const std::vector<int>& segments,
                                const std::string& filename,
                                const Eigen::Matrix3d& rotation,
                                std::map<int, Eigen::Vector3i>* color_table);

void WriteOtherPointsWithColor(const std::vector<Point>& points,
                               const std::vector<int>& segments,
                               const std::string& filename,
                               const Eigen::Matrix3d& rotation);
    
 
}  // namespace structured_indoor_modeling

#endif  // OBJECT_SEGMENTATION_H_
