#pragma once

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

#include "../base/floorplan.h"

namespace structured_indoor_modeling {

class IndoorPolygon;
class ViewParameters;
  
class IndoorPolygonRenderer : protected QGLFunctions {
 public:
  IndoorPolygonRenderer(const IndoorPolygon& indoor_polygon);
  virtual ~IndoorPolygonRenderer();
  void RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha) const;
  void RenderTextureMappedRooms(const double top_alpha,
                                const double bottom_alpha,
                                const ViewParameters& view_parameters,
                                const double air_to_tree_progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift) const;
  
  void Init(const std::string& data_directory,
            const std::string& suffix,
            QGLWidget* widget);
  void InitGL();
  
 private:
  QGLWidget* widget;
  const IndoorPolygon& indoor_polygon;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;

  // Floor outline.
  std::map<int, std::vector<std::vector<Eigen::Vector3d> > > wire_frames;

  double bottom_z;
  double top_z;
};

}  // namespace structured_indoor_modeling
