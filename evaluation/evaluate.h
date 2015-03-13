#include <Eigen/Dense>
#include <vector>

namespace structured_indoor_modeling {

enum GeometryType {
  kFloorplanFloor,
  kFloorplanCeiling,
  kFloorplanWall,
  kFloorplanDoor,

  kIndoorPolygonFloor,
  kIndoorPolygonCeiling,
  kIndoorPolygonWall,
  kIndoorPolygonDoor,
  
  kObject,

  kHole
};
  
struct RasterizedGeometry {
  RasterizedGeometry(const double depth,
                     const Eigen::Vector3d& normal,
                     const GeometryType& geometry_type)
  : depth(depth), normal(normal), geometry_type(geometry_type) {
  }
  double depth;
  Eigen::Vector3d normal;
  GeometryType geometry_type;
};  

class FileIO;
class Floorplan;
class IndoorPolygon;
class Panorama;
class PointCloud;

void ReadInputPointClouds(const FileIO& file_io,
                          std::vector<PointCloud>* input_point_clouds);
void ReadObjectPointClouds(const FileIO& file_io,
                           const int num_rooms,
                           std::vector<PointCloud>* object_point_clouds);
void ReadPanoramas(const FileIO& file_io,
                   std::vector<Panorama>* panoramas);

Eigen::Vector3d GetLaserCenter(const FileIO& file_io, const int panorama);

void Initialize(const std::vector<Panorama>& panoramas,
                const RasterizedGeometry& initial_value,
                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries);

void RasterizeFloorplan(const Floorplan& floorplan,
                        const std::vector<Panorama>& panoramas,
                        std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries);

void RasterizeIndoorPolygon(const IndoorPolygon& indoor_polygon,
                            const std::vector<Panorama>& panoramas,
                            std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries);

void RasterizeObjectPointClouds(const std::vector<PointCloud>& object_point_clouds,
                                const std::vector<Panorama>& panoramas,
                                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries);

void ReportErrors(const std::vector<PointCloud>& input_point_clouds,
                  const std::vector<std::vector<RasterizedGeometry> >& rasterized_geometries,
                  const std::vector<Panorama>& panoramas,
                  const RasterizedGeometry& initial_value);

 
}  // namespace structured_indoor_modeling
  
