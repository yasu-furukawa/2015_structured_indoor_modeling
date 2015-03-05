#pragma once

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

#include "../base/floorplan.h"

namespace structured_indoor_modeling {

class IndoorPolygonRenderer : protected QGLFunctions {
 public:
  IndoorPolygonRenderer(const IndoorPolygon& indoor_polygon);
  virtual ~IndoorPolygonRenderer();
  void RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha) const;
  
  void Init(const std::string data_directory, QGLWidget* widget);
  void InitGL();
  
 private:
  QGLWidget* widget;
  const IndoorPolygon& indoor_polygon;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;
};

}  // namespace structured_indoor_modeling
