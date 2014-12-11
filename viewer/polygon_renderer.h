#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>

#include "../floorplan/floorplan.h"
class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer();
  virtual ~PolygonRenderer();
  void RenderWallAll(const Eigen::Vector3d& center,
                     const double alpha,
                     const double height_adjustment,
                     const int center_room,
                     const int room_highlighted,
                     const bool render_room_id);
  void RenderWireframeAll(const double alpha);
  void RenderWireframe(const int room, const double alpha);
  void Init(const std::string data_directory);
  // void InitGL();

  const LineFloorplan& GetLineFloorplan() const { return line_floorplan; }
  const Eigen::Matrix3d& GetFloorplanToGlobal() const { return floorplan_to_global; }
  Eigen::Vector2d GetRoomCenter(const int room) const { return room_centers[room]; }
  Eigen::Vector3d GetRoomCenterGlobal(const int room) const {
    const Eigen::Vector2d center = GetRoomCenter(room);
    return floorplan_to_global *
      Eigen::Vector3d(center[0],
                      center[1],
                      (line_floorplan.line_rooms[room].floor_height + 
                       line_floorplan.line_rooms[room].ceiling_height) / 2.0);
  }
  
 private:  
  LineFloorplan line_floorplan;
  Eigen::Matrix3d floorplan_to_global;

  std::vector<Eigen::Vector2d> room_centers;
};

#endif  // POLYGON_RENDERER_H__
