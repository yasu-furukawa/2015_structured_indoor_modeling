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

inline std::istream& operator>>(std::istream& istr, Triangle& triangle) {
  for (int i = 0; i < 3; ++i) {
    istr >> triangle.indices[i];
  }
  istr >> triangle.image_index;
  for (int i = 0; i < 3; ++i) {
    istr >> triangle.uvs[i][0] >> triangle.uvs[i][1];
  }
  return istr;
}

inline std::ostream& operator<<(std::ostream& ostr, const Triangle& triangle) {
  for (int i = 0; i < 3; ++i) {
    ostr << triangle.indices[i] << ' ';
  }
  ostr << triangle.image_index << ' ';
  for (int i = 0; i < 3; ++i) {
    ostr << triangle.uvs[i][0] << ' ' << triangle.uvs[i][1] << ' ';
  }
  ostr << std::endl;
  return ostr;
}
 
// std::istream& operator>>(std::istream& istr, Triangle& triangle);
// std::ostream& operator<<(std::ostream& ostr, const Triangle& triangle);
 
}  // namespace structured_indoor_modeling
