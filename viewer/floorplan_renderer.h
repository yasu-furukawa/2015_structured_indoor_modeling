#ifndef FLOORPLAN_RENDERER_H__
#define FLOORPLAN_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>
#include <string>
#include <vector>

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <windows.h>
#include <GL/glu.h>
//#ifndef __glew_h__
//#include <GL/glew.h>
//#include <GL/glext.h>
//#endif
#else
#include <OpenGL/glu.h>
#endif

namespace structured_indoor_modeling {

class Floorplan;

struct PaintStyle {
public:
  enum FillStyle {
    SolidColor,
    VerticalStripe,
    WaterDrop,
    Sheep,
    Kitchen,
    Tile
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
  void InitGL(QGLWidget* widget_tmp);
  void RenderLabels();
  void Render(const double alpha,
              const GLint viewport[],
              const GLdouble modelview[],
              const GLdouble projection[],
              const bool emphasize);

 private:
  PaintStyle GetPaintStyle(const std::vector<std::string>& room_name) const;

  void RenderRoomFill(const int room,
                      const double unit,
                      const PaintStyle& paint_style,
                      const double alpha,
                      const bool set_stencil) const;

  void RenderRoomStroke(const int room,
                        const PaintStyle& paint_style,
                        const double alpha,
                        const bool emphasize) const;


  void RenderSolidColor(const int room,
                        const PaintStyle& paint_style,
                        const double unit,
                        const double alpha) const;

  void RenderVerticalStripe(const int room,
                            const PaintStyle& paint_style,
                            const double unit,
                            const double alpha) const;

  void RenderWaterDrop(const int room,
                       const PaintStyle& paint_style,
                       const double unit,
                       const double alpha) const;

  void RenderSheep(const int room,
                   const PaintStyle& paint_style,
                   const double unit,
                   const double alpha) const;

  void RenderKitchen(const int room,
                     const PaintStyle& paint_style,
                     const double unit,
                     const double alpha) const;

  void RenderTile(const int room,
                  const PaintStyle& paint_style,
                  const double unit,
                  const double alpha) const;

  void RenderTexture(const int room,
                     const PaintStyle& paint_style,
                     const double unit,
                     const double alpha,
                     const GLint texture_id,
                     const double texture_scale) const;
  
  void RenderDoor(const Vector3d& lhs, const Vector3d& rhs) const;
  
  //Floorplan floorplan;
  QGLWidget* widget;
  const Floorplan& floorplan;
  QImage sheep_image;
  QImage kitchen_image;
  QImage tile_image;
  GLint sheep_texture_id;
  GLint kitchen_texture_id;  
  GLint tile_texture_id;

  const GLint* viewport;
  const GLdouble* modelview;
  const GLdouble* projection;
};
 
}  // namespace structured_indoor_modeling
 
#endif  // FLOORPLAN_RENDERER_H__
