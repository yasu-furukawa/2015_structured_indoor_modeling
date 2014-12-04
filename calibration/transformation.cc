#include "transformation.h"

using namespace Eigen;

Matrix3d RotationX(const double rx) {
  Matrix3d rotation;
  rotation << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
  return rotation;
}

Matrix3d RotationY(const double ry) {
  Matrix3d rotation;
  rotation << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
  return rotation;
}

Matrix3d RotationZ(const double rz) {
  Matrix3d rotation;
  rotation << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
  return rotation;
}

void ConvertLocalToPanorama(const int panorama_width, const int panorama_height,
                            const double phi_per_pixel, const Vector3d& ray,
                            Vector2d* uv) {
  Vector3d normalized_ray = ray.normalized();
  const double phi = asin(normalized_ray[2]);
  double theta = -atan2(ray[1], ray[0]);
  if (theta < 0.0)
    theta += 2 * M_PI;

  (*uv)[0] = theta / (2 * M_PI) * panorama_width;
  (*uv)[1] = panorama_height / 2.0 - phi / phi_per_pixel;
}

void ConvertPanoramaToLocal(const int panorama_width, const int panorama_height,
                       const double phi_per_pixel, const Vector2d& uv,
                       Vector3d* ray) {
  const double theta = uv[0] * 2 * M_PI / panorama_width;
  const double phi = (panorama_height / 2.0 - uv[1]) * phi_per_pixel;
  (*ray)[2] = sin(phi);
  (*ray)[1] = sin(-theta) * cos(phi);
  (*ray)[0] = cos(-theta) * cos(phi);
}
