#pragma once

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

#include "../base/floorplan.h"

namespace structured_indoor_modeling {

class IndoorPolygon;
class TreeOrganizer;
  
class IndoorPolygonRenderer : protected QGLFunctions {
 public:
  IndoorPolygonRenderer(const IndoorPolygon& indoor_polygon);
  virtual ~IndoorPolygonRenderer();
  void RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha) const;
  void RenderTextureMappedRooms(const double top_alpha,
                                const double bottom_alpha,
                                const TreeOrganizer& tree_organizer,
                                const double air_to_tree_progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift,
                                const double max_shrink_ratio) const;
  
  void Init(const std::string data_directory, QGLWidget* widget);
  void InitGL();
  
 private:
  QGLWidget* widget;
  const IndoorPolygon& indoor_polygon;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;

  double bottom_z;
  double top_z;
};

}  // namespace structured_indoor_modeling
