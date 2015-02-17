/*
  A point cloud class with read and write capabilities. Each point
  can have many information beyond its 3D position. struct Point has
  the list.

  There are several utility functions also implemented. For example,
  one can transform the point information to the global coordinate
  system by ToGlobal(). This is the typical suggested usage. Read data
  from FileIO::GetLocalPly(const int panorama), and convert them to
  the global coordinate frame.

  Rotate() and Transform() can apply arbitrary
  transformations. SetPoints and AddPoints can change the stored point
  cloud data. This class has been mostly used for read-only access,
  and the implemented features are fairly limited at the moment.

  < Example >

  FileIO file_io("../some_data_directory");
  PointCloud point_cloud;
  const int kPanoramaID = 3;
  point_cloud.Init(file_io, kPanoramaID);

  PointCloud point_cloud2;
  point_cloud2.Init(file_io.GetObjectCloudsWithColor());

  PointCloud point_cloud3;
  point_cloud3.AddPoints(point_cloud);
  point_cloud3.AddPoints(point_cloud2);
  point_cloud3.Write("new_file.ply");
 */
#ifndef BASE_POINT_CLOUD_H_
#define BASE_POINT_CLOUD_H_

#include <vector>

namespace structured_indoor_modeling {

class FileIO;

struct Point {
  Eigen::Vector2i depth_position;
  Eigen::Vector3d position;
  Eigen::Vector3f color;
  Eigen::Vector3d normal;
  int intensity;
  int object_id;
  Point(){object_id = -1;}
};

class PointCloud {
 public:
  PointCloud();
  // Read the corresponding point cloud in the local coordinate frame.
  bool Init(const FileIO& file_io, const int panorama);
  // Read the point cloud with the given filename.
  bool Init(const std::string& filename);
  // Writer.
  void Write(const std::string& filename);

  // Transformations.
  void ToGlobal(const FileIO& file_io, const int panorama);
  void Rotate(const Eigen::Matrix3d& rotation);
  void Transform(const Eigen::Matrix4d& transformation);

  // Accessors.
  int GetNumPoints() const { return points.size(); }
  int GetDepthWidth() const { return depth_width; }
  int GetDepthHeight() const { return depth_height; }
  int GetNumObjects() const { return num_object; }
  const Point& GetPoint(const int p) const { return points[p]; }
  Point& GetPoint(const int p) { return points[p]; }

  // Setters.
  void SetPoints(const std::vector<Point>& new_points) {
    points = new_points;
  }
  void AddPoints(const PointCloud& point_cloud) {
    points.insert(points.end(), point_cloud.points.begin(), point_cloud.points.end());
  }
  void SetNum_Object(int num){
    num_object = num;
  }

 private:
  std::vector<Point> points;
  int depth_width;
  int depth_height;
  int num_object;
};

typedef std::vector<PointCloud> PointClouds;

}  // namespace structured_indoor_modeling

#endif  // BASE_POINT_CLOUD_H_
