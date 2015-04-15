#ifndef OBJECT_RENDERER_H_
#define OBJECT_RENDERER_H_

#include <Eigen/Dense>
#include <QGLFunctions>
#include <fstream>
#include <vector>
#include <string>
#include <QGLFunctions>

#include "../base/detection.h"

namespace structured_indoor_modeling {

class Floorplan;
class IndoorPolygon;
class Navigation;
class ViewParameters;

typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> ColoredPoint;
typedef std::vector<ColoredPoint> ColoredPointCloud;

struct BoundingBox2D {
  Eigen::Vector3d corners[4];
};

class ObjectRenderer : protected QGLFunctions {
 public:
   ObjectRenderer(const Floorplan& floorplan,
                  const IndoorPolygon& indoor_polygon,
		  const Navigation& navigation,
                  const std::string& detection_file);
  virtual ~ObjectRenderer();

  void Init(const std::string data_directory);
  void InitGL();

  void RenderAll(const double position);
  void RenderAll(const ViewParameters& view_parameters,
                 const double air_to_tree_progress,
                 const double animation,
                 const Eigen::Vector3d& offset_direction);

  void RenderIcons(const double alpha,
                   const double animation,
                   const GLint viewport[],
                   const GLdouble modelview[],
                   const GLdouble projection[],
                   QGLWidget* widget);

  int GetNumRooms() const;
  int GetNumObjects(const int room) const;
  const std::vector<float>& GetObject(const int room, const int object) const;

  void Precompute(const ViewParameters& view_parameters);
  bool Toggle();
  
private:
  void ComputeBoundingBoxes2D();
  void RenderRectangle(const Detection& detection,
                       const Eigen::Vector3d bounding_boxes[4],
                       const double animation,
                       const Eigen::Vector3f& color) const;
  void RenderObjectPolygon(const Detection& detection,
			   const Eigen::Vector3f& color) const;
  void RenderDesk(const Detection& detection,
                  const Eigen::Vector3d bounding_boxes[4],
                  const double animation) const;
  void RenderSofa(const Detection& detection,
                  const Eigen::Vector3d vs[4],
                  const double animation) const;
  void RenderChair(const Detection& detection,
                   const Eigen::Vector3d bounding_boxes[4],
                   const double animation) const;
  void RenderLamp(const Detection& detection,
                  const Eigen::Vector3d vs[4],
                  const double animation) const;
  void RenderDefault(const Detection& detection,
                     const Eigen::Vector3d vs[4],
                     const double animation) const;
  
  void RotateToFrontal(const Detection& detection,
                       Eigen::Vector3d vs[4]) const;

  static void MakeSquare(Eigen::Vector3d vs[4]);
  
  void RenderName(const Detection& detection,
                  const Eigen::Vector3d vs[4],
                  const double animation,
                  const GLint viewport[],
                  const GLdouble modelview[],
                  const GLdouble projection[],                  
                  QGLWidget* widget) const;
  
  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const Navigation& navigation;
  // private:
  bool render;
  // For each room, for each object, a colored point cloud.
  // std::vector<std::vector<ColoredPointCloud> > colored_point_clouds;

  double average_floor_height;
  double distance_per_pixel;

  std::vector<std::vector<std::vector<float> > > vertices;
  std::vector<std::vector<std::vector<float> > > colors;
  //object center
  std::vector<std::vector<Eigen::Vector3d> > centers;

  std::vector<std::vector<std::vector<float> > > vertices_org;
  std::vector<std::vector<std::vector<float> > > colors_org;

  // Precomputed information.
  std::vector<std::vector<std::vector<float> > > vertices_middle;
  std::vector<std::vector<std::vector<float> > > vertices_bottom;


  // Bounding boxes for each object.
  std::vector<std::vector<BoundingBox2D> > bounding_boxes_2D;

  std::vector<Detection> detections;
  
};

}  // namespace structured_indoor_modeling 

#endif  // OBJECT_RENDERER_H_
