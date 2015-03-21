#ifndef DATA_H_
#define DATA_H_

#include <Eigen/Dense>
#include <vector>

namespace structured_indoor_modeling {

struct OrientedPoint {
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
  // Angular resolution is not equal in a depth scan.
  float weight;
};

struct Sweep {
  Eigen::Vector3d center;
  std::vector<OrientedPoint> points;
};

struct Frame {
  Eigen::Vector3d axes[3];
  Eigen::Vector2f ranges[3];
  Eigen::Vector3i size;
  float unit;

  Eigen::Vector3d ToLocalPosition(const Eigen::Vector3d& position) const;
  Eigen::Vector3d ToLocalNormal(const Eigen::Vector3d& normal) const;
};

std::istream& operator>>(std::istream& istr, Frame& frame);
std::ostream& operator<<(std::ostream& ostr, const Frame& frame);

Eigen::Vector3d ConvertPoint(const Eigen::Vector3d& point,
                             const Eigen::Matrix4d& transformation);

Eigen::Vector3d ConvertNormal(const Eigen::Vector3d& normal,
                              const Eigen::Matrix4d& transformation);

}  // namespace structured_indoor_modeling

#endif  // DATA_H_
