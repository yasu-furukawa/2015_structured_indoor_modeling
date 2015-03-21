#ifndef FLOORED_DATA_H_
#define FLOORED_DATA_H_

#include <Eigen/Dense>

namespace floored {

struct OrientedPoint {
  Eigen::Vector3f position;
  Eigen::Vector3f normal;
  // Angular resolution is not equal in a depth scan.
  float weight;
};

struct Sweep {
  Eigen::Vector3f center;
  std::vector<OrientedPoint> points;
};

struct Frame {
  Eigen::Vector3f axes[3];
  Eigen::Vector2f ranges[3];
  Eigen::Vector3i size;
  float unit;

  Eigen::Vector3f ToLocalPosition(const Eigen::Vector3f& position) const {
    return Eigen::Vector3f((position.dot(axes[0]) - ranges[0][0]) / unit,
                           (position.dot(axes[1]) - ranges[1][0]) / unit,
                           (position.dot(axes[2]) - ranges[2][0]) / unit);
  }
  Eigen::Vector3f ToLocalNormal(const Eigen::Vector3f& normal) const {
    return Eigen::Vector3f(normal.dot(axes[0]),
                           normal.dot(axes[1]),
                           normal.dot(axes[2])).normalized();
  }
};

struct Floorplan {
  enum SpaceType{
    Inside,
    Outside
  };

  int width;
  int height;
  std::vector<SpaceType> in_out;
};
 
inline std::istream& operator>>(std::istream& istr, Frame& frame) {
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

inline std::ostream& operator<<(std::ostream& ostr, const Frame& frame) {
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
 
}  // namespace floored

#endif  // FLOORED_DATA_H_
