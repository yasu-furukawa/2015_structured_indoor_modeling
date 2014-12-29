#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

#include "../floorplan/floorplan.h"
class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer();
  virtual ~PolygonRenderer();
  void RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha);
  void RenderWallAll(const Eigen::Vector3d& center,
                     const double alpha,
                     const double height_adjustment,
                     const bool depth_order_height_adjustment,
                     const int room_not_rendered,
                     const int room_highlighted,
                     const bool render_room_id);
  void RenderWireframeAll(const double alpha);
  void RenderWireframe(const int room, const double alpha);
  void Init(const std::string data_directory, QGLWidget* widget);
  void InitGL();

  const Floorplan& GetFloorplan() const { return floorplan; }
  Eigen::Vector2d GetRoomCenter(const int room) const { return room_centers_local[room]; }
  Eigen::Vector3d GetRoomCenterGlobal(const int room) const {
    const Eigen::Vector2d center = GetRoomCenter(room);
    return floorplan.GetFloorplanToGlobal() *
      Eigen::Vector3d(center[0],
                      center[1],
                      (floorplan.GetFloorHeight(room) +
                       floorplan.GetCeilingHeight(room)) / 2.0);
  }
  Eigen::Vector3d GetRoomCenterFloorGlobal(const int room) const {
    const Eigen::Vector2d center = GetRoomCenter(room);
    return floorplan.GetFloorplanToGlobal() *
      Eigen::Vector3d(center[0],
                      center[1],
                      floorplan.GetFloorHeight(room));
  }
  
 private:  
  QGLWidget* widget;
  Floorplan floorplan;

  std::vector<Eigen::Vector2d> room_centers_local;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;
};

#endif  // POLYGON_RENDERER_H__
