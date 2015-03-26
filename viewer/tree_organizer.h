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

  // Displacement to the target (horizontal motion).
  std::vector<Eigen::Vector3d> displacements;
};
 
struct ObjectDeformation {
  std::vector<std::vector<BoundingBox> > bounding_boxes;

  // Displacement to the target (horizontal motion).
  std::vector<Eigen::Vector3d> displacements;
};
  
class TreeOrganizer {
 public:
  TreeOrganizer(const Floorplan& floorplan,
                const IndoorPolygon& indoor_polygon,
                const ObjectRenderer& object_renderer);
  void Init(const Eigen::Vector3d& tree_layout_direction,
            const Eigen::Vector3d& tree_layout_orthogonal_direction);

  const FloorplanDeformation& GetFloorplanDeformation() const;
  const FloorplanDeformation& GetIndoorPolygonDeformation() const;
  const ObjectDeformation& GetObjectDeformation() const;

 private:
  void InitFloorplanDeformation();
  void InitIndoorPolygonDeformation();
  void InitObjectDeformation();

  void ComputeDisplacementsFloorplan(const Eigen::Vector3d& tree_layout_direction,
                                     const Eigen::Vector3d& tree_layout_orthogonal_direction);

  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const ObjectRenderer& object_renderer;

  FloorplanDeformation floorplan_deformation;
  FloorplanDeformation indoor_polygon_deformation;
  ObjectDeformation object_deformation;
  

};
  
}  // namespace structured_indoor_modeling
