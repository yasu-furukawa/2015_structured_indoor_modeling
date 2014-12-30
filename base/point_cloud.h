#ifndef BASE_POINT_CLOUD_H_
#define BASE_POINT_CLOUD_H_

#include <vector>
#include "../calibration/file_io.h"

namespace base {

struct Point {
  Eigen::Vector2i depth_position;
  Eigen::Vector3d position;
  Eigen::Vector3f color;
  Eigen::Vector3d normal;
  int intensity;
};

class PointCloud {
 public:
  PointCloud();
  bool Init(const file_io::FileIO& file_io, const int panorama);
  void ToGlobal(const file_io::FileIO& file_io, const int panorama);

  // void Write(const string filename) const;

  int GetNumPoints() const { return points.size(); }
  int GetDepthWidth() const { return depth_width; }
  int GetDepthHeight() const { return depth_height; }
  const Point& GetPoint(const int p) const { return points[p]; }
  Point& GetPoint(const int p) { return points[p]; }

 private:
  std::vector<Point> points;
  int depth_width;
  int depth_height;
};

typedef std::vector<PointCloud> PointClouds;

}  // namespace base

#endif  // BASE_POINT_CLOUD_H_
