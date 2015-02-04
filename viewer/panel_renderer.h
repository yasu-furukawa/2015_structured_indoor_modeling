#ifndef PANEL_RENDERER_H_
#define PANEL_RENDERER_H_

#include <QGLFunctions>
#include <QGLWidget>
#include <QImage>
#include <Eigen/Dense>
#include <vector>

namespace structured_indoor_modeling {

class Floorplan;
class MainWidget;

class PanelRenderer : protected QGLFunctions {
 public:
  PanelRenderer(const Floorplan& floorplan,
                const GLint* viewport);
  virtual ~PanelRenderer();
  void Init(const std::string& data_directory);
  void InitGL(MainWidget* main_widget);
  void RenderThumbnail(const double alpha,
                       const int room_highlighted,
                       const Eigen::Vector2i& render_pos,
                       const Eigen::Vector3d& color,
                       const double scale,
                       QGLWidget* qgl_widget);
const QImage& GetRoomThumbnail(const int room) { return room_thumbnails[room]; }

 static const double kWidthRatio;
 static const int kTextHeight;
 static const int kFrameMargin;

 private:
  std::vector<QImage> room_thumbnails;
  const Floorplan& floorplan;
  const GLint* viewport;
  GLuint thumbnail_texid;
};

}  // namespace structured_indoor_modeling

#endif  // PANEL_RENDERER_H_
