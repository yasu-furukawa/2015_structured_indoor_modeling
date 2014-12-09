#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"

class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer();
  virtual ~PolygonRenderer();
  void RenderWallAll();
  void RenderWall(const int room);
  void RenderWireframeAll(const double alpha);
  void RenderWireframe(const int room, const double alpha);
  void Init(const std::string data_directory);
  // void InitGL();

 private:  
  LineFloorplan line_floorplan;
  Eigen::Matrix3d rotation;
};

#endif  // POLYGON_RENDERER_H__
