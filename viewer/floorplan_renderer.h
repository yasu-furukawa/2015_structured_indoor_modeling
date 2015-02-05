#ifndef FLOORPLAN_RENDERER_H__
#define FLOORPLAN_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

namespace structured_indoor_modeling {

class Floorplan;

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
  FloorplanRenderer(const Floorplan& floorplan);
  virtual ~FloorplanRenderer();
  void Init();
  void Render(const double alpha) const;

 private:
  /*
  void RenderShape(const Shape& shape,
                   const double floor_height,
                   const PolygonStyle& style,
                   const double alpha);
  */
  
  //Floorplan floorplan;
  const Floorplan& floorplan;
};

}  // namespace structured_indoor_modeling
 
#endif  // FLOORPLAN_RENDERER_H__
