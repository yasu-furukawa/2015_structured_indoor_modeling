#ifndef FLOORPLAN_RENDERER_H__
#define FLOORPLAN_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>
#include <string>
#include <vector>

namespace structured_indoor_modeling {

class Floorplan;

struct PaintStyle {
public:
  enum FillStyle {
    SolidColor,
    VerticalStripe,
    WaterDrop,
    Sheep
  };

  FillStyle fill_style;
  Eigen::Vector3f fill_color;
  Eigen::Vector3f stroke_color;
  double stroke_width;

PaintStyle(const PaintStyle::FillStyle fill_style,
           const Eigen::Vector3f& fill_color,
           const Eigen::Vector3f& stroke_color,
           const double stroke_width) :
    fill_style(fill_style),
    fill_color(fill_color),
    stroke_color(stroke_color),
    stroke_width(stroke_width) {
  }
};

class FloorplanRenderer : protected QGLFunctions {
 public:
  FloorplanRenderer(const Floorplan& floorplan);
  virtual ~FloorplanRenderer();
  void Init();
  void Render(const double alpha) const;

 private:
  PaintStyle GetPaintStyle(const std::vector<std::string>& room_name) const;

  void RenderRoomFill(const int room,
                      const double unit,
                      const PaintStyle& paint_style,
                      const double alpha,
                      const bool set_stencil) const;
  void RenderRoomStroke(const int room,
                        const PaintStyle& paint_style,
                        const double alpha) const;
  
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
