#ifndef POLYGON_RENDERER_H__
#define POLYGON_RENDERER_H__

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

#include "../base/floorplan.h"

namespace structured_indoor_modeling {

class TreeOrganizer;

struct Triangle2 {
  Eigen::Vector3d vertices[3];
  Eigen::Vector3d colors[3];
  Eigen::Vector3i colori;
};

typedef std::vector<Triangle2> Triangles;

class PolygonRenderer : protected QGLFunctions {
 public:
  PolygonRenderer(const Floorplan& floorplan);
  virtual ~PolygonRenderer();
  void RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha) const;
  void RenderTextureMappedRooms(const double top_alpha,
                                const double bottom_alpha,
                                const TreeOrganizer& tree_organizer,
                                const double air_to_tree_progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift,
                                const double max_shrink_ratio) const;
  void RenderDoors(const double alpha) const;
  void RenderDoors(const double alpha,
                   const TreeOrganizer& tree_organizer,
                   const double air_to_tree_progress,
                   const double animation,
                   const Eigen::Vector3d& max_vertical_shift,
                   const double max_shrink_ratio) const;
  
  void RenderWallAll(const Eigen::Vector3d& center,
                     const double alpha,
                     const double height_adjustment,
                     const bool depth_order_height_adjustment,
                     const int room_not_rendered,
                     const int room_highlighted,
                     const bool render_room_id);

  void RenderColoredBoxes(const Eigen::Vector3d& max_vertical_shift,
                          const double max_shrink_scale,
                          const double air_to_tree_progress,
                          const double animation,
                          const double alpha,
                          const Eigen::Vector3d& room_center,
                          const Eigen::Vector3d& center);
  
  void RenderWireframeAll(const double alpha);
  void RenderWireframe(const int room, const double alpha);
  void Init(const std::string data_directory, QGLWidget* widget);
  void InitGL();
  
 private:
  void SetTargetCeilingHeights(const Eigen::Vector3d& center,
                               const bool depth_order_height_adjustment,
                               const int room_not_rendered,
                               std::vector<double>* target_ceiling_heights);
  void AddTrianglesFromWalls(const double height_adjustment,
                             const int room_not_rendered,
                             const int room_highlighted,
                             const std::vector<double>& target_ceiling_heights,
                             Triangles* triangles);
  void AddTrianglesFromCeiling(const double height_adjustment,
                               const int room_not_rendered,
                               const int room_highlighted,
                               const std::vector<double>& target_ceiling_heights,
                               Triangles* triangles);

  void AddTrianglesFromWalls(Triangles* triangles) const;
  void AddTrianglesFromCeiling(Triangles* triangles) const;
  void AddTrianglesFromDoors(Triangles* triangles) const;
  
  QGLWidget* widget;
  const Floorplan& floorplan;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;
};

}  // namespace structured_indoor_modeling
 
#endif  // POLYGON_RENDERER_H__
