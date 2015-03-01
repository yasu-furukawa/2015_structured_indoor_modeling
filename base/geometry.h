#pragma once

#include <Eigen/Dense>

namespace structured_indoor_modeling {

struct Triangle {
  // Vertex indices.
  Eigen::Vector3i indices;
  // Texture image index.
  int image_index;
  // UV coordinate.
  Eigen::Vector2d uvs[3];
};

std::istream& operator>>(std::istream& istr, Triangle& triangle);
std::ostream& operator<<(std::ostream& ostr, const Triangle& triangle);
 
}  // namespace structured_indoor_modeling
