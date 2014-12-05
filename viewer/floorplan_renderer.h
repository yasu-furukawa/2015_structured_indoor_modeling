#ifndef FLOORPLAN_RENDERER_H__
#define FLOORPLAN_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"

struct FloorplanStyle {
public:
  Eigen::Vector3i outer_line_color;
  Eigen::Vector3i outer_line_color;
  

};

class FloorplanRenderer : protected QGLFunctions {
 public:
  FloorplanRenderer();
  virtual ~FloorplanRenderer();
  void Init(const std::string data_directory);
  // void InitGL();
  void Render();

 private:
  void RenderShape(const Shape& shape, const Vector3i& line_color);
  
  Floorplan floorplan;
  Eigen::Matrix3d rotation;
};

#endif  // FLOORPLAN_RENDERER_H__
