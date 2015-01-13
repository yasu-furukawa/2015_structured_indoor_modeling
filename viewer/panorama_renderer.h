#ifndef PANORAMA_RENDERER_H__
#define PANORAMA_RENDERER_H__

#include <vector>
#include <Eigen/Dense>
#include <QImage>
#include <QGLFunctions>
#include <QGLWidget>

#include "configuration.h"

namespace structured_indoor_modeling {

class PanoramaRenderer : protected QGLFunctions {
 public:
  PanoramaRenderer();
  virtual ~PanoramaRenderer();
  void Render(const double alpha);
  void Init(const PanoramaConfiguration& panorama_configuration, QGLWidget* widget);
  void InitGL();
  Eigen::Vector2d Project(const Eigen::Vector3d& xyz) const;
  Eigen::Vector3d Unproject(const Eigen::Vector2d& uv, const double distance) const;

  Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global_xyz) const;
  Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local_xyz) const;

  const Eigen::Vector3d& GetCenter() const { return center; }
  double GetAverageDistance() const { return average_distance; }
  Eigen::Vector2d RGBToDepth(const Eigen::Vector2d& pixel) const;

  const std::vector<Eigen::Vector3d>& DepthMesh() const { return depth_mesh; }
  int DepthWidth() const { return depth_width; }
  int DepthHeight() const { return depth_height; }
  
 private:
  void InitDepthMesh(const std::string& filename, const double phi_range);

  // For texture allocation and deletion.
  QGLWidget* widget;
  
  // Image.
  QImage rgb_image;
  GLint texture_id;

  // Depthmap is turned into a grid mesh.
  int depth_width;
  int depth_height;
  std::vector<Eigen::Vector3d> depth_mesh;
  // Average distance in the depthmap.
  double average_distance;

  // Camera position.
  Eigen::Vector3d center;
  // Camera rotation.
  Eigen::Matrix3d local_to_global;
  Eigen::Matrix3d global_to_local;
  // Camera intrinsics: phi (in radian) per image y.
  float phi_per_pixel;
  float depth_phi_per_pixel;

};

}  // namespace structured_indoor_modeling
 
#endif  // PANORAMA_RENDERER_H__
