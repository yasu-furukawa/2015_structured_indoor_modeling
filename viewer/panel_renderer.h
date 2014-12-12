#ifndef PANEL_RENDERER_H_
#define PANEL_RENDERER_H_

#include <QGLFunctions>
#include <QImage>
#include <Eigen/Dense>
#include <vector>

class MainWidget;
class PolygonRenderer;

class PanelRenderer : protected QGLFunctions {
 public:
  PanelRenderer(const PolygonRenderer& polygon_renderer,
                const GLint* viewport);
  virtual ~PanelRenderer();
  void Init(const std::string& data_directory);
  void InitGL(MainWidget* main_widget);
  void RenderThumbnail(const double alpha,
                       const int room_highlighted,
                       const Eigen::Vector2i& render_pos,
                       const Eigen::Vector3d& color,
                       const double scale,
                       MainWidget* main_widget);
const QImage& GetRoomThumbnail(const int room) { return room_thumbnails[room]; }

  static const double kWidthRatio = 0.2;
  static const int kTextHeight = 6;
  static const int kFrameMargin = 5;

 private:
  std::vector<QImage> room_thumbnails;
  const PolygonRenderer& polygon_renderer;  
  const GLint* viewport;
  GLuint thumbnail_texid;
};

#endif  // PANEL_RENDERER_H_
