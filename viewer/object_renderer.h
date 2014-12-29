#ifndef OBJECT_RENDERER_H_
#define OBJECT_RENDERER_H_

#include <Eigen/Dense>
#include <vector>

#include "../calibration/file_io.h"

typedef std::pair<Vector3d, Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

class ObjectRenderer : protected QGLFunctions {
 public:
  ObjectRenderer();
  virtual ~ObjectRenderer();

  void Init(const file_io::FileIO& file_io);

  void RenderAll();
  void RenderRoom(const int room);
  void RenderObject(const int room, const int object);

 private:
  // For each room, for each object, a colored point cloud.
  std::vector<std::vector<ColoredPointCloud> > colored_point_cloud;
};

#endif  // OBJECT_RENDERER_H_
