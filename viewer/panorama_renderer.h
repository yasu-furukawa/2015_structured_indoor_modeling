#ifndef PANORAMA_RENDERER_H__
#define PANORAMA_RENDERER_H__

#include <vector>
#include <Eigen/Dense>
#include <QImage>
#include <QGLFunctions>
#include <QGLWidget>

#include "configuration.h"

namespace structured_indoor_modeling {

class Panorama;

class PanoramaRenderer : protected QGLFunctions {
 public:
  PanoramaRenderer();
  virtual ~PanoramaRenderer();
  void Render(const double alpha) const;
  // void Init(const PanoramaConfiguration& panorama_configuration, QGLWidget* widget);
  void Init(const FileIO& file_io, const int panorama_id, const Panorama* panorama, QGLWidget* widget);
  void InitGL();

  const std::vector<Eigen::Vector3d>& DepthMesh() const { return depth_mesh; }
  int DepthWidth() const { return depth_width; }
  int DepthHeight() const { return depth_height; }
  const Panorama& GetPanorama() const { return *panorama; }
  
 private:
  void InitDepthMesh(const std::string& filename, const double phi_range);

  // Without texture data.
  const Panorama* panorama;
  // For texture allocation and deletion.
  QGLWidget* widget;
  
  // Image.
  QImage rgb_image;
  GLint texture_id;

  // Depthmap is turned into a grid mesh.
  int depth_width;
  int depth_height;
  std::vector<Eigen::Vector3d> depth_mesh;
};

}  // namespace structured_indoor_modeling
 
#endif  // PANORAMA_RENDERER_H__
