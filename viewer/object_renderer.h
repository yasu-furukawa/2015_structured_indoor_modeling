#ifndef OBJECT_RENDERER_H_
#define OBJECT_RENDERER_H_

#include <Eigen/Dense>
#include <QGLFunctions>
#include <fstream>
#include <vector>
#include <string>

namespace structured_indoor_modeling {

class Floorplan;

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

class ObjectRenderer : protected QGLFunctions {
 public:
  ObjectRenderer(const Floorplan& floorplan);
  virtual ~ObjectRenderer();

  void Init(const std::string data_directory);
  void InitGL();

  void RenderAll(const double alpha);
  void RenderRoom(const int room) const;
  void RenderObject(const int room, const int object) const;

  bool Toggle();
  
 private:
  const Floorplan& floorplan;
  // private:
  bool render;
  // For each room, for each object, a colored point cloud.
  std::vector<std::vector<ColoredPointCloud> > colored_point_clouds;

  std::vector<float> vertices;
  std::vector<float> colors;
};

}  // namespace structured_indoor_modeling 

#endif  // OBJECT_RENDERER_H_
