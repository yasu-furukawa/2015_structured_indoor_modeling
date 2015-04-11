#pragma once

#include <Eigen/Dense>
#include <QGLFunctions>
#include <QImage>

namespace structured_indoor_modeling {

class Floorplan;
class IndoorPolygon;
class Navigation;
class ViewParameters;
 
class IndoorPolygonRenderer : protected QGLFunctions {
 public:
  enum RenderMode {
    kFull,
    kBackWallFaceCulling,
    kBackWallFaceTransparent
  };
  
  IndoorPolygonRenderer(const Floorplan& floorplan,
                        const IndoorPolygon& indoor_polygon,
                        const Navigation& navigation);
  virtual ~IndoorPolygonRenderer();
  void RenderTextureMappedRooms(const double top_intensity, const double bottom_intensity);
  void RenderTextureMappedRooms(const double top_intensity,
                                const double bottom_intensity,
                                const ViewParameters& view_parameters,
                                const double air_to_tree_progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift) const;
  
  void Init(const std::string& data_directory,
            const std::string& suffix,
            QGLWidget* widget);
  void InitGL();
  void ToggleRenderMode();
  RenderMode GetRenderMode() const { return render_mode; }
  
 private:
  QGLWidget* widget;
  const Floorplan& floorplan;
  const IndoorPolygon& indoor_polygon;
  const Navigation& navigation;

  std::vector<QImage> texture_images;
  std::vector<GLint> texture_ids;

  // Floor outline.
  std::map<int, std::vector<std::vector<Eigen::Vector3d> > > wire_frames;

  RenderMode render_mode;
  
  double bottom_z;
  double top_z;
};

}  // namespace structured_indoor_modeling
