#include <Eigen/Dense>
#include <vector>
#include <string>

namespace structured_indoor_modeling {

enum GeometryType {
  kFloor,
  kCeiling,
  kWall,
  kDoor,

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

struct Mesh {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> faces;
  GeometryType geometry_type;
};
 
class FileIO;
class Floorplan;
class IndoorPolygon;
class Panorama;
class PointCloud;

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

bool ReadMesh(const std::string& filename, Mesh* mesh);
bool ReadMeshAscii(const std::string& filename, Mesh* mesh);
bool ReadMeshBinary(const std::string& filename, Mesh* mesh);

void RasterizeMesh(const Mesh& mesh,
                   const std::vector<Panorama>& panoramas,
                   std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries);

void ReportErrors(const FileIO& file_io,
                  const std::string& prefix,
                  const std::vector<PointCloud>& input_point_clouds,
                  const std::vector<std::vector<RasterizedGeometry> >& rasterized_geometries,
                  const std::vector<Panorama>& panoramas,
                  const RasterizedGeometry& initial_value,
                  const double depth_unit);
 
}  // namespace structured_indoor_modeling
  
