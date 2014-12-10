#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"

class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer();
  virtual ~PolygonRenderer();
  void RenderWallAll(const Eigen::Vector3d& center,
                     const double alpha,
                     const double height_adjustment,
                     const int center_room);
  void RenderWall(const int room);
  void RenderWireframeAll(const double alpha);
  void RenderWireframe(const int room, const double alpha);
  void Init(const std::string data_directory);
  // void InitGL();

  const LineFloorplan& GetLineFloorplan() { return line_floorplan; }
  const Eigen::Matrix3d& GetFloorplanToGlobal() { return floorplan_to_global; }

 private:  
  LineFloorplan line_floorplan;
  Eigen::Matrix3d floorplan_to_global;

  std::vector<Eigen::Vector2d> room_centers;
};

#endif  // POLYGON_RENDERER_H__
