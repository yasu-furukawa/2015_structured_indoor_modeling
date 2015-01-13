#include "data.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

Eigen::Vector3d Frame::ToLocalPosition(const Eigen::Vector3d& position) const {
  return Eigen::Vector3d((position.dot(axes[0]) - ranges[0][0]) / unit,
                         (position.dot(axes[1]) - ranges[1][0]) / unit,
                         (position.dot(axes[2]) - ranges[2][0]) / unit);
}

Eigen::Vector3d Frame::ToLocalNormal(const Eigen::Vector3d& normal) const {
  return Eigen::Vector3d(normal.dot(axes[0]),
                         normal.dot(axes[1]),
                         normal.dot(axes[2])).normalized();
}

std::istream& operator>>(std::istream& istr, Frame& frame) {
  std::string header;
  istr >> header;
  for (int a = 0; a < 3; ++a)
    for (int i = 0; i < 3; ++i)
      istr >> frame.axes[a][i];

  for (int a = 0; a < 3; ++a)
    for (int i = 0; i < 2; ++i)
      istr >> frame.ranges[a][i];

  for (int i = 0; i < 3; ++i)
    istr >> frame.size[i];

  istr >> frame.unit;

  return istr;
};

std::ostream& operator<<(std::ostream& ostr, const Frame& frame) {
  ostr << "FRAME" << std::endl;
  for (int a = 0; a < 3; ++a) {
    for (int i = 0; i < 3; ++i) {
      ostr << frame.axes[a][i] << ' ';
    }
    ostr << std::endl;
  }

  for (int a = 0; a < 3; ++a) {
    for (int i = 0; i < 2; ++i) {
      ostr << frame.ranges[a][i] << ' ';
    }
    ostr << std::endl;
  }

  for (int i = 0; i < 3; ++i)
    ostr << frame.size[i] << ' ';
  ostr << std::endl;

  ostr << frame.unit << std::endl;

  return ostr;
};

Eigen::Vector3d ConvertPoint(const Eigen::Vector3d& point,
                             const Eigen::Matrix4d& transformation) {
  Vector4d point4(point[0], point[1], point[2], 1.0);
  point4 = transformation * point4;
  return Vector3d(point4[0], point4[1], point4[2]);
}

Eigen::Vector3d ConvertNormal(const Eigen::Vector3d& normal,
                              const Eigen::Matrix4d& transformation) {
  Vector4d normal4(normal[0], normal[1], normal[2], 0.0);
  normal4 = transformation * normal4;
  return Vector3d(normal4[0], normal4[1], normal4[2]);
}

}  // namespace structured_indoor_modeling
