#ifndef FLOORPLAN_RENDERER_H__
#define FLOORPLAN_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"

struct PolygonStyle {
public:
  Eigen::Vector3f stroke_color;
  Eigen::Vector3f fill_color;
  int stroke_width;
};

struct FloorplanStyle {
public:
  PolygonStyle outer_style;
  PolygonStyle inner_style;
};

class FloorplanRenderer : protected QGLFunctions {
 public:
  FloorplanRenderer();
  virtual ~FloorplanRenderer();
  void Init(const std::string data_directory);
  // void InitGL();
  void Render(const FloorplanStyle& style, const double alpha);

 private:
  static void RenderShape(const Shape& shape,
                          const double floor_height,
                          const Eigen::Matrix3d& rotation,
                          const PolygonStyle& style,
                          const double alpha);
  
  Floorplan floorplan;
  Eigen::Matrix3d rotation;
};

#endif  // FLOORPLAN_RENDERER_H__
