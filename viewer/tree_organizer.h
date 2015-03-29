#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "object_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct BoundingBox {
  BoundingBox() {
    min_xyz = Eigen::Vector3d(std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max());
    max_xyz = Eigen::Vector3d(- std::numeric_limits<double>::max(),
                              - std::numeric_limits<double>::max(),
                              - std::numeric_limits<double>::max());
  }

  Eigen::Vector3d min_xyz;
  Eigen::Vector3d max_xyz;
};

// All local.
 struct TreeConfiguration {
   // Before bounding box.
   BoundingBox bounding_box;
   // Before center.
   Eigen::Vector3d center;

   // Offset with respect to the node one higher up (local)
   Eigen::Vector3d displacement;
   // Scale.
   double scale;
   
   // For object row.
   int row;
 };

class TreeOrganizer {
 public:
  TreeOrganizer(const Floorplan& floorplan,
                const IndoorPolygon& indoor_polygon,
                const ObjectRenderer& object_renderer);
  void Init(const Eigen::Vector3d& x_axis,
            const Eigen::Vector3d& y_axis,
            const Eigen::Vector3d& z_axis);

  Eigen::Vector3d TransformRoom(const Vector3d& global,
                                const int room,
                                const int wall,
                                const double progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift) const;
  
  Eigen::Vector3d TransformObject(const Vector3d& global,
                                  const int room,
                                  const int object,
                                  const double progress,
                                  const double animation,
                                  const Eigen::Vector3d& room_max_vertical_shift,
                                  const Eigen::Vector3d& object_max_vertical_shift) const;

 private:
  void InitCenter();
  void InitBoundingBoxes();
  void InitTreeConfigurationCenter();
  void SetDisplacements();
  
  Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global) const {
    const Eigen::Vector3d& diff = global - center;
    return Eigen::Vector3d(x_axis.dot(diff), y_axis.dot(diff), z_axis.dot(diff));
  }
  Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local) const {
    return local[0] * x_axis + local[1] * y_axis + local[2] * z_axis + center;
  }
  Eigen::Vector3d GlobalToLocalNormal(const Eigen::Vector3d& global) const {
    return Eigen::Vector3d(x_axis.dot(global), y_axis.dot(global), z_axis.dot(global));
  }
  Eigen::Vector3d LocalToGlobalNormal(const Eigen::Vector3d& local) const {
    return local[0] * x_axis + local[1] * y_axis + local[2] * z_axis;
  }
  
  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const ObjectRenderer& object_renderer;
  // global.
  Eigen::Vector3d center;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  Eigen::Vector3d z_axis;

  // Displacement is from the current room location.
  std::vector<TreeConfiguration> room_configurations;
  // Displacement is from the moved room.
  std::vector<TreeConfiguration> floor_configurations;
  // Displacement is from the moved room.
  std::vector<std::vector<TreeConfiguration> > wall_configurations;
  // Displacement is from the moved room.  
  std::vector<std::vector<TreeConfiguration> > object_configurations;  
};
  
}  // namespace structured_indoor_modeling
