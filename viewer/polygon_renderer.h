#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"

struct Wall {
  Eigen::Vector3d corners[4];
};

class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer();
  virtual ~PolygonRenderer();
  void RenderWallAll();
  void RenderWall(const int room);
  void RenderWireframeAll();
  void RenderWireframe(const int room);
  void Init(const std::string data_directory);
  // void InitGL();

 private:
  /*
  void SortWalls(const Eigen::Vector3d& center,
                 const Eigen::Vector3d& direction,
                 vector<Wall>* walls);
  */
  
  LineFloorplan line_floorplan;
  Eigen::Matrix3d rotation;
};

#endif  // POLYGON_RENDERER_H__
