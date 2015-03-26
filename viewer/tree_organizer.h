#pragma once

#include <Eigen/Dense>
#include <vector>
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "object_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct FloorplanCenters {
  std::vector<Eigen::Vector3d> room_centers;
  std::vector<Eigen::Vector3d> floor_centers;
  std::vector<std::vector<Eigen::Vector3d> > wall_centers;
};
 
struct ObjectCenters {
  std::vector<std::vector<Eigen::Vector3d> > centers;
};
  
class TreeOrganizer {
 public:
  TreeOrganizer(const Floorplan& floorplan,
                const IndoorPolygon& indoor_polygon,
                const ObjectRenderer& object_renderer);

  void Init();

 private:
  void InitFloorplanCenters();
  void InitIndoorPolygonCenters();
  void InitObjectCenters();

  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const ObjectRenderer& object_renderer;

  FloorplanCenters floorplan_centers;
  FloorplanCenters indoor_polygon_centers;
  ObjectCenters object_centers;
  

};
  
}  // namespace structured_indoor_modeling
