#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "object_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct FloorplanDeformation {
  std::vector<Eigen::Vector3d> room_centers;
  std::vector<Eigen::Vector3d> floor_centers;
  std::vector<std::vector<Eigen::Vector3d> > wall_centers;
};
 
struct ObjectDeformation {
  std::vector<std::vector<Eigen::Vector3d> > centers;
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
