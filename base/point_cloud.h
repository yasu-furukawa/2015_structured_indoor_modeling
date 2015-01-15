#ifndef BASE_POINT_CLOUD_H_
#define BASE_POINT_CLOUD_H_

#include <vector>
#include "file_io.h"

namespace structured_indoor_modeling {

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
  bool Init(const FileIO& file_io, const int panorama);
  bool Init(const std::string& filename);
  void ToGlobal(const FileIO& file_io, const int panorama);
  void Rotate(const Eigen::Matrix3d& rotation);
  void Transform(const Eigen::Matrix4d& transformation);

  void Write(const std::string& filename);

  // void Write(const string filename) const;

  int GetNumPoints() const { return points.size(); }
  int GetDepthWidth() const { return depth_width; }
  int GetDepthHeight() const { return depth_height; }
  const Point& GetPoint(const int p) const { return points[p]; }
  Point& GetPoint(const int p) { return points[p]; }

  void SetPoints(const std::vector<Point>& new_points) {
    points = new_points;
  }

 private:
  std::vector<Point> points;
  int depth_width;
  int depth_height;
};

typedef std::vector<PointCloud> PointClouds;

}  // namespace structured_indoor_modeling

#endif  // BASE_POINT_CLOUD_H_
