#ifndef TRANSFORMATION_H__
#define TRANSFORMATION_H__

#include <Eigen/dense>

Eigen::Matrix3d RotationX(const double rx);
Eigen::Matrix3d RotationY(const double ry);
Eigen::Matrix3d RotationZ(const double rz);

void ConvertLocalToPanorama(const int panorama_width, const int panorama_height,
                            const double phi_per_pixel, const Eigen::Vector3d& ray,
                            Eigen::Vector2d* uv);

void ConvertPanoramaToLocal(const int panorama_width, const int panorama_height,
                            const double phi_per_pixel, const Eigen::Vector2d& uv,
                            Eigen::Vector3d* ray);

#endif  // TRANSFORMATION_H__
