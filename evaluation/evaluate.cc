#include <fstream>
#include <iostream>
#include "evaluate.h"
#include "../base/file_io.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct Mesh {
  std::vector<Vector3d> vertices;
  std::vector<Vector3i> faces;
};
  
namespace {

int GetEndPanorama(const FileIO& file_io, const int start_panorama) {
  int panorama = start_panorama;
  while (1) {
    const string filename = file_io.GetPanoramaImage(panorama);
    ifstream ifstr;
    ifstr.open(filename.c_str());
    if (!ifstr.is_open()) {
      ifstr.close();
      return panorama;
    }
    ifstr.close();
    ++panorama;
  }
}

void RasterizeMesh(const Panorama& panorama,
                   const Mesh& mesh,
                   std::vector<RasterizedGeometry>* rasterized_geometry) {
  /*                 
  const double kInvalidDepth = numeric_limits<double>::max();
  DepthPixel invalid_depth_pixel;
  invalid_depth_pixel.depth = kInvalidDepth;
  invalid_depth_pixel.normal = Vector3d(0, 0, 0);
  
  vector<DepthPixel> depths(camera.width * camera.height,
                            invalid_depth_pixel);

  // Rasterize.
  for (const auto& mesh : meshes) {
    int count = 0;
    for (const auto& triangle : mesh.triangles) {
      ++count;
      if (count % (mesh.triangles.size() / 10) == 0)
        cerr << '.' << flush;
      
      const Vector3d vs[3] = { mesh.vertices[triangle[0]],
                               mesh.vertices[triangle[1]],
                               mesh.vertices[triangle[2]] };
      const Vector3d center = (vs[0] + vs[1] + vs[2]) / 3.0;
      const double unit = min(min(ComputeUnit(camera, vs[0]),
                                  ComputeUnit(camera, vs[1])),
                              min(ComputeUnit(camera, vs[2]),
                                  ComputeUnit(camera, center)));

      Vector3d normal = - (vs[1] - vs[0]).cross(vs[2] - vs[0]);
      if (normal.norm() == 0) {
        continue;
      }
      normal.normalize();
      
      const double kSampleScale = 2.0;
      const int sample01 = max(2, static_cast<int>(round((vs[1] - vs[0]).norm() / unit * kSampleScale)));
      const int sample02 = max(2, static_cast<int>(round((vs[2] - vs[0]).norm() / unit * kSampleScale)));
      const int sample_s = max(sample01, sample02);
      for (int s = 0; s <= sample_s; ++s) {
        const Vector3d v01 = vs[0] + (vs[1] - vs[0]) * s / sample_s;
        const Vector3d v02 = vs[0] + (vs[2] - vs[0]) * s / sample_s;
        const int sample_t = max(2, static_cast<int>(round((v02 - v01).norm() / unit * kSampleScale)));
        for (int t = 0; t <= sample_t; ++t) {
          const Vector3d point = v01 + (v02 - v01) * t / sample_t;
          const double distance = (point - camera.center).norm();
          Vector2d uv = Project(camera, point);
          const int u = static_cast<int>(round(uv[0])) % camera.width;
          const int v = static_cast<int>(round(uv[1]));

          const int index = v * camera.width + u;
          if (distance < depths[index].depth) {
            depths[index].depth = distance;
            depths[index].normal = normal;
          }
        }
      }
    }
    cerr << endl;
  }
  for (int x = 0; x < camera.width; ++x) {
    const int index0 = 0 * camera.width + x;
    const int index1 = (camera.height - 1) * camera.width + x;
    depths[index0].depth = kInvalidDepth;
    depths[index1].depth = kInvalidDepth;
  }
  */
}
  
}  // namespace  

void ReadInputPointClouds(const FileIO& file_io, std::vector<PointCloud>* input_point_clouds) {
  const int kStartPanorama = 0;
  const int end_panorama = GetEndPanorama(file_io, kStartPanorama);

  input_point_clouds->clear();
  input_point_clouds->resize(end_panorama);
  {
    cout << "Reading point clouds..." << flush;
    for (int p = kStartPanorama; p < end_panorama; ++p) {
      cout << '.' << flush;
      if (!input_point_clouds->at(p).Init(file_io, p)) {
        cerr << "Failed in loading the point cloud." << endl;
        exit (1);
      }
      input_point_clouds->at(p).ToGlobal(file_io, p);
    }
  }
}

void ReadObjectPointClouds(const FileIO& file_io,
                           const int num_rooms,
                           std::vector<PointCloud>* object_point_clouds) {
  object_point_clouds->clear();
  object_point_clouds->resize(num_rooms);
  for (int room = 0; room < num_rooms; ++room) {
    object_point_clouds->at(room).Init(file_io.GetRefinedObjectClouds(room));
  }
}

void ReadPanoramas(const FileIO& file_io,
                   vector<Panorama>* panoramas) {
  const int kStartPanorama = 0;
  const int end_panorama = GetEndPanorama(file_io, kStartPanorama);
  panoramas->clear();
  panoramas->resize(end_panorama);
  for (int p = kStartPanorama; p < end_panorama; ++p) {
    panoramas->at(p).Init(file_io, p);
  }
}

Vector3d GetLaserCenter(const FileIO& file_io, const int panorama) {
  ifstream ifstr;
  ifstr.open(file_io.GetLocalToGlobalTransformation(panorama).c_str());
  
  char ctmp;
  ifstr >> ctmp;
  Vector3d center;
  for (int i = 0; i < 3; ++i) {
    double dtmp;
    for (int x = 0; x < 3; ++x)
      ifstr >> dtmp;
    ifstr >> center[i];
  }
  ifstr.close();

  return center;
}

void Initialize(const vector<Panorama>& panoramas,
                const RasterizedGeometry& initial_value,
                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  rasterized_geometries->clear();
  rasterized_geometries->resize(panoramas.size());

  for (int p = 0; p < (int)panoramas.size(); ++p) {
    const int width  = panoramas[p].DepthWidth();
    const int height = panoramas[p].DepthHeight();

    rasterized_geometries->at(p).resize(width * height, initial_value);
  }
}

void RasterizeFloorplan(const Floorplan& floorplan,
                        const vector<Panorama>& panoramas,
                        std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  for (int p = 0; p < panoramas.size(); ++p) {
    const Panorama& panorama = panoramas[p];
    vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries->at(p);

    // Floor.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const FloorCeilingTriangulation& triangulation = floorplan.GetFloorTriangulation(room);
      Mesh mesh;
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        mesh.vertices.push_back(floorplan.GetFloorVertexGlobal(room, v));
      }
      for (const auto& triangle : triangulation.triangles) {
        mesh.faces.push_back(triangle.indices);
      }
      RasterizeMesh(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
    // Ceiling.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const FloorCeilingTriangulation& triangulation = floorplan.GetCeilingTriangulation(room);
      Mesh mesh;
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        mesh.vertices.push_back(floorplan.GetCeilingVertexGlobal(room, v));
      }
      for (const auto& triangle : triangulation.triangles) {
        mesh.faces.push_back(triangle.indices);
      }
      RasterizeMesh(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
    // Walls.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        Mesh mesh;
        for (int v = 0; v < floorplan.GetNumWallVertices(room, wall); ++v)
          mesh.vertices.push_back(floorplan.GetWallVertexGlobal(room, wall, v));

        for (int t = 0; t < floorplan.GetNumWallTriangles(room, wall); ++t) {
          mesh.faces.push_back(floorplan.GetWallTriangle(room, wall, t).indices);
        }
        RasterizeMesh(panoramas[p], mesh, &rasterized_geometries->at(p));
      }
    }
    // Doors.
    for (int door = 0; door < floorplan.GetNumDoors(); ++door) {
      Mesh mesh;
      for (int v = 0; v < floorplan.GetNumDoorVertices(door); ++v) {
        mesh.vertices.push_back(floorplan.GetDoorVertexGlobal(door, v));
      }
      for (int t = 0; t < floorplan.GetNumDoorTriangles(door); ++t) {
        mesh.faces.push_back(floorplan.GetDoorTriangle(door, t).indices);
      }
      RasterizeMesh(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
  }
}

void RasterizeIndoorPolygon(const IndoorPolygon& indoor_polygon,
                            const vector<Panorama>& panoramas,
                            std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {


}

void RasterizeObjectPointClouds(const std::vector<PointCloud>& object_point_clouds,
                                const vector<Panorama>& panoramas,
                                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {


}

void ReportErrors(const std::vector<PointCloud>& input_point_clouds,
                  const std::vector<std::vector<RasterizedGeometry> >& rasterized_geometries,
                  const vector<Panorama>& panoramas) {



}
  
}  // namespace structured_indoor_modeling
