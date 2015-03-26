#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "object_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct BoundingBox {
  Eigen::Vector3d min_xyz;
  Eigen::Vector3d max_xyz;
};
  
struct FloorplanDeformation {
  std::vector<BoundingBox> room_bounding_boxes;
  std::vector<BoundingBox> floor_bounding_boxes;
  std::vector<std::vector<BoundingBox> > wall_bounding_boxes;
};
 
struct ObjectDeformation {
  std::vector<std::vector<BoundingBox> > bounding_boxes;
};
  
class TreeOrganizer {
 public:
  TreeOrganizer(const Floorplan& floorplan,
                const IndoorPolygon& indoor_polygon,
                const ObjectRenderer& object_renderer);
  void Init();

  const FloorplanDeformation& GetFloorplanDeformation() const;
  const FloorplanDeformation& GetIndoorPolygonDeformation() const;
  const ObjectDeformation& GetObjectDeformation() const;

 private:
  void InitFloorplanDeformation();
  void InitIndoorPolygonDeformation();
  void InitObjectDeformation();

  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const ObjectRenderer& object_renderer;

  FloorplanDeformation floorplan_deformation;
  FloorplanDeformation indoor_polygon_deformation;
  ObjectDeformation object_deformation;
  

};
  
}  // namespace structured_indoor_modeling
