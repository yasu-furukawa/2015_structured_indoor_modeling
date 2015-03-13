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

#include <Eigen/Dense>
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
  Point(){object_id = 0;}
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
  inline int GetNumPoints() const { return points.size(); }
  inline int GetDepthWidth() const { return depth_width; }
  inline int GetDepthHeight() const { return depth_height; }
  // yasu This should return const reference to speed-up.
  inline const std::vector<double>& GetBoundingbox() const { return bounding_box; }
  inline int GetNumObjects() const { return num_objects; }
  // yasu This should return const reference to speed-up.
  inline const Eigen::Vector3d& GetCenter() const { return center; }
  inline const Point& GetPoint(const int p) const { return points[p]; }
  inline Point& GetPoint(const int p) { return points[p]; }
  double GetBoundingboxVolume();  
  // Setters.
  void SetPoints(const std::vector<Point>& new_points) {
    points = new_points;
    Update();
  }
  
  void SetAllColor(float r,float g,float b);
  void SetColor(int ind, float r, float g,float b);

  void AddPoints(const PointCloud& point_cloud);
  void AddPoints(const std::vector<Point>& new_points);

  void RemovePoints(const std::vector<int>& indexes);
  
 private:
  void InitializeMembers();
  void Update();

  std::vector<Point> points;

  //Since deleting element on vector is inefficient, we set up this mask array.
  // yasu This should be vector<bool>.
  Eigen::Vector3d center;
  int depth_width;
  int depth_height;
  int num_objects;
  std::vector <double> bounding_box; //xmin,xmax,ymin,ymax,zmin,zmax

  static const int kDepthPositionOffset;
};

typedef std::vector<PointCloud> PointClouds;

}  // namespace structured_indoor_modeling

#endif  // BASE_POINT_CLOUD_H_
