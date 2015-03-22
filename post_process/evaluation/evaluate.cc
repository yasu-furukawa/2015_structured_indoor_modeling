#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include "evaluate.h"
#include "../../base/file_io.h"
#include "../../base/floorplan.h"
#include "../../base/indoor_polygon.h"
#include "../../base/panorama.h"
#include "../../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct Triple {
  Eigen::Vector3d points[3];
};
  
namespace {

double ComputeUnit(const Panorama& panorama,
                   const Vector3d& point) {
  const Vector2d uv_depth = panorama.ProjectToDepth(point);
  const double distance = (panorama.GetCenter() - point).norm();
  const Vector2d right_depth(uv_depth[0] + 1.0, uv_depth[1]);
  const Vector2d right = panorama.DepthToRGB(right_depth);
  const Vector3d right_point = panorama.Unproject(right, distance);
  return (point - right_point).norm();  
}

void PaintPoint(const Panorama& panorama,
                const Eigen::Vector3d& point,
                const Eigen::Vector3d& normal,
                const GeometryType& geometry_type,
                std::vector<RasterizedGeometry>* rasterized_geometry) {
  const int width  = panorama.DepthWidth();
  const int height = panorama.DepthHeight();

  const double distance = (point - panorama.GetCenter()).norm();
  Vector2d uv = panorama.ProjectToDepth(point);
  const int u = static_cast<int>(round(uv[0])) % width;
  const int v = min(height - 1, static_cast<int>(round(uv[1])));

  const int index = v * width + u;
  if (distance < rasterized_geometry->at(index).depth) {
    rasterized_geometry->at(index).depth = distance;
    rasterized_geometry->at(index).normal = normal;
    rasterized_geometry->at(index).geometry_type = geometry_type;
  }
}

double UVDistance(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs, const int width) {
  double u_diff = fabs(lhs[0] - rhs[0]);
  double v_diff = fabs(lhs[1] - rhs[1]);
  
  if (u_diff > width / 2)
    u_diff = width - u_diff;

  const double distance = sqrt(u_diff * u_diff + v_diff * v_diff);
  // cout << distance << ' ' << flush;
  return distance;
}

void RasterizeMeshForPanorama(const Panorama& panorama,
                              const Mesh& mesh,
                              std::vector<RasterizedGeometry>* rasterized_geometry) {
  const int width = panorama.DepthWidth();
  const int height = panorama.DepthHeight();

  const double kSmallestSize = 1.0;
  
  for (int f = 0; f < mesh.faces.size(); ++f) {
    const Vector3i& triangle = mesh.faces[f];
    const Vector3d vs[3] = { mesh.vertices[triangle[0]],
                             mesh.vertices[triangle[1]],
                             mesh.vertices[triangle[2]] };

    Vector3d normal = (vs[1] - vs[0]).cross(vs[2] - vs[0]);
    if (normal.norm() == 0) {
      continue;
    }
    normal.normalize();

    const Vector3d center = (vs[0] + vs[1] + vs[2]) / 3.0;
    PaintPoint(panorama, center, normal, mesh.geometry_type, rasterized_geometry);

    list<Triple> queue;
    {
      Triple initial_triple;
      for (int i = 0; i < 3; ++i)
        initial_triple.points[i] = vs[i];
      queue.push_back(initial_triple);
    }

    while (!queue.empty()) {
      const Triple triple = queue.front();
      queue.pop_front();

      const Vector2d uv0 = panorama.ProjectToDepth(triple.points[0]);
      const Vector2d uv1 = panorama.ProjectToDepth(triple.points[1]);
      const Vector2d uv2 = panorama.ProjectToDepth(triple.points[2]);
      if ((uv0[1] < 0.5 && uv1[1] < 0.5 && uv2[1] < 0.5) ||
          (uv0[1] > height - 1.5 && uv1[1] > height - 1.5 && uv2[1] > height - 1.5))
        continue;

      if (UVDistance(uv0, uv1, width) < kSmallestSize &&
          UVDistance(uv1, uv2, width) < kSmallestSize &&
          UVDistance(uv2, uv0, width) < kSmallestSize)
        continue;

      // Recurse.
      const Vector3d edge01 = (triple.points[0] + triple.points[1]) / 2.0;
      const Vector3d edge12 = (triple.points[1] + triple.points[2]) / 2.0;
      const Vector3d edge20 = (triple.points[2] + triple.points[0]) / 2.0;

      PaintPoint(panorama, (triple.points[0] + edge01 + edge20) / 3.0, normal, mesh.geometry_type,
                 rasterized_geometry);
      PaintPoint(panorama, (triple.points[1] + edge12 + edge01) / 3.0, normal, mesh.geometry_type,
                 rasterized_geometry);
      PaintPoint(panorama, (triple.points[2] + edge20 + edge12) / 3.0, normal, mesh.geometry_type,
                 rasterized_geometry);
      {
        Triple triple0;
        triple0.points[0] = triple.points[0];
        triple0.points[1] = edge01;
        triple0.points[2] = edge20;
        queue.push_front(triple0);
      }
      {
        Triple triple1;
        triple1.points[0] = triple.points[1];
        triple1.points[1] = edge12;
        triple1.points[2] = edge01;
        queue.push_front(triple1);
      }
      {
        Triple triple2;
        triple2.points[0] = triple.points[2];
        triple2.points[1] = edge20;
        triple2.points[2] = edge12;
        queue.push_front(triple2);
      }
      {
        Triple triple3;
        triple3.points[0] = edge01;
        triple3.points[1] = edge12;
        triple3.points[2] = edge20;
        queue.push_front(triple3);
      }
    }
  }
  
  /*
  const int width = panorama.DepthWidth();
  const int height = panorama.DepthHeight();
  
  for (int f = 0; f < mesh.faces.size(); ++f) {
    const Vector3i& triangle = mesh.faces[f];
    const Vector3d vs[3] = { mesh.vertices[triangle[0]],
                             mesh.vertices[triangle[1]],
                             mesh.vertices[triangle[2]] };
    const Vector3d center = (vs[0] + vs[1] + vs[2]) / 3.0;
    const double unit = min(min(ComputeUnit(panorama, vs[0]),
                                ComputeUnit(panorama, vs[1])),
                            min(ComputeUnit(panorama, vs[2]),
                                ComputeUnit(panorama, center)));

    Vector3d normal = (vs[1] - vs[0]).cross(vs[2] - vs[0]);
    if (normal.norm() == 0) {
      continue;
    }
    normal.normalize();

    // 6.0 is too much and slow, but things work.
    const double kSampleScale = 12.0;
    const int sample01 = max(2, static_cast<int>(round((vs[1] - vs[0]).norm() / unit * kSampleScale)));
    const int sample02 = max(2, static_cast<int>(round((vs[2] - vs[0]).norm() / unit * kSampleScale)));
    const int sample_s = max(sample01, sample02);
    for (int s = 0; s <= sample_s; ++s) {
      const Vector3d v01 = vs[0] + (vs[1] - vs[0]) * s / sample_s;
      const Vector3d v02 = vs[0] + (vs[2] - vs[0]) * s / sample_s;
      const int sample_t = max(2, static_cast<int>(round((v02 - v01).norm() / unit * kSampleScale)));

      for (int t = 0; t <= sample_t; ++t) {
        const Vector3d point = v01 + (v02 - v01) * t / sample_t;
        const double distance = (point - panorama.GetCenter()).norm();
        Vector2d uv = panorama.ProjectToDepth(point);
        const int u0 = static_cast<int>(floor(uv[0])) % width;
        const int v0 = static_cast<int>(floor(uv[1]));

        for (int j = 0; j < 2; ++j) {
          const int vtmp = min(height - 1, (v0 + j));
          for (int i = 0; i < 2; ++i) {
            const int utmp = (u0 + i) % width;

            const int index = vtmp * width + utmp;
            if (distance < rasterized_geometry->at(index).depth) {
              rasterized_geometry->at(index).depth = distance;
              rasterized_geometry->at(index).normal = normal;
              rasterized_geometry->at(index).geometry_type = mesh.geometry_type;
            }
          }
        }
      }

    }
  }
  */  
}
  
}  // namespace  

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
  cout << "RasterizeFloorplan." << endl;
  for (int p = 0; p < panoramas.size(); ++p) {
    cout << "Panorama " << p << '/' << panoramas.size() << flush;
    const Panorama& panorama = panoramas[p];
    vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries->at(p);

    cout << " floor" << flush;
    // Floor.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const FloorCeilingTriangulation& triangulation = floorplan.GetFloorTriangulation(room);
      Mesh mesh;
      mesh.geometry_type = kFloor;
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        mesh.vertices.push_back(floorplan.GetFloorVertexGlobal(room, v));
      }
      for (const auto& triangle : triangulation.triangles) {
        mesh.faces.push_back(triangle.indices);
      }
      RasterizeMeshForPanorama(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
    cout << " ceiling" << flush;
    // Ceiling.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const FloorCeilingTriangulation& triangulation = floorplan.GetCeilingTriangulation(room);
      Mesh mesh;
      mesh.geometry_type = kCeiling;
      for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
        mesh.vertices.push_back(floorplan.GetCeilingVertexGlobal(room, v));
      }
      for (const auto& triangle : triangulation.triangles) {
        mesh.faces.push_back(triangle.indices);
      }
      RasterizeMeshForPanorama(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
    cout << " walls" << flush;
    // Walls.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        Mesh mesh;
        mesh.geometry_type = kWall;
        
        for (int v = 0; v < floorplan.GetNumWallVertices(room, wall); ++v) {
          mesh.vertices.push_back(floorplan.GetWallVertexGlobal(room, wall, v));
        }
        for (int t = 0; t < floorplan.GetNumWallTriangles(room, wall); ++t) {
          mesh.faces.push_back(floorplan.GetWallTriangle(room, wall, t).indices);
        }
        RasterizeMeshForPanorama(panoramas[p], mesh, &rasterized_geometries->at(p));
      }
    }
    cout << " doors" << flush;
    // Doors.
    for (int door = 0; door < floorplan.GetNumDoors(); ++door) {
      Mesh mesh;
      mesh.geometry_type = kDoor;

      for (int v = 0; v < floorplan.GetNumDoorVertices(door); ++v) {
        mesh.vertices.push_back(floorplan.GetDoorVertexGlobal(door, v));
      }
      for (int t = 0; t < floorplan.GetNumDoorTriangles(door); ++t) {
        mesh.faces.push_back(floorplan.GetDoorTriangle(door, t).indices);
      }
      RasterizeMeshForPanorama(panoramas[p], mesh, &rasterized_geometries->at(p));
    }
    cout << " done." << endl;
  }
}

void RasterizeIndoorPolygon(const IndoorPolygon& indoor_polygon,
                            const vector<Panorama>& panoramas,
                            std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  cout << "RasterizeIndoorPolygon." << endl;
  for (int p = 0; p < panoramas.size(); ++p) {
    cout << "Panorama " << p << '/' << panoramas.size() << endl;
    const Panorama& panorama = panoramas[p];
    vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries->at(p);
    for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
      const Segment& segment = indoor_polygon.GetSegment(s);
      Mesh mesh;
      switch (segment.type) {
      case Segment::FLOOR: {
        mesh.geometry_type = kFloor;
        break;
      }
      case Segment::CEILING: {
        mesh.geometry_type = kCeiling;
        break;
      }
      case Segment::WALL: {
        mesh.geometry_type = kWall;
        break;
      }
      case Segment::DOOR: {
        mesh.geometry_type = kDoor;
        break;
      }
      }

      for (int v = 0; v < segment.vertices.size(); ++v) {
        mesh.vertices.push_back(indoor_polygon.ManhattanToGlobal(segment.vertices[v]));
      }
      for (const auto& triangle : segment.triangles) {
        mesh.faces.push_back(triangle.indices);
      }
      
      RasterizeMeshForPanorama(panorama, mesh, &rasterized_geometry);
    }
  }
}

void RasterizeObjectPointClouds(const std::vector<PointCloud>& object_point_clouds,
                                const vector<Panorama>& panoramas,
                                std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  for (int p = 0; p < panoramas.size(); ++p) {
    cout << "Panorama " << p << '/' << panoramas.size() << endl;
    const Panorama& panorama = panoramas[p];
    vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries->at(p);

    for (const auto& point_cloud : object_point_clouds) {
      for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
        const Point& point = point_cloud.GetPoint(p);
        PaintPoint(panorama, point.position, point.normal, kObject, &rasterized_geometry);
      }
    }
  }
}

void ReadMesh(const std::string& filename, Mesh* mesh) {
  ifstream ifstr;
  ifstr.open(filename.c_str());

  //????
  

  ifstr.close();

  mesh->geometry_type = kWall;
}

void RasterizeMesh(const Mesh& mesh,
                   const std::vector<Panorama>& panoramas,
                   std::vector<std::vector<RasterizedGeometry> >* rasterized_geometries) {
  for (int p = 0; p < panoramas.size(); ++p) {
    cout << "Panorama " << p << '/' << panoramas.size() << endl;
    const Panorama& panorama = panoramas[p];
    vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries->at(p);
    RasterizeMeshForPanorama(panorama, mesh, &rasterized_geometry);
  }
}
  
void ReportErrors(const std::vector<PointCloud>& input_point_clouds,
                  const std::vector<std::vector<RasterizedGeometry> >& rasterized_geometries,
                  const vector<Panorama>& panoramas,
                  const RasterizedGeometry& initial_value,
                  const double depth_unit) {
  vector<map<GeometryType, pair<Vector2d, int> > > errors(panoramas.size());
  const pair<Vector2d, int> kInitial(Vector2d(0, 0), 0);
  vector<GeometryType> all_geometry_types;
  {
    all_geometry_types.push_back(kFloor);
    all_geometry_types.push_back(kCeiling);
    all_geometry_types.push_back(kWall);
    all_geometry_types.push_back(kDoor);
    all_geometry_types.push_back(kObject);
    all_geometry_types.push_back(kHole);
  }
  for (int p = 0; p < (int)panoramas.size(); ++p) {
    for (const auto& type : all_geometry_types) {
      errors[p][type]  = kInitial;
    }
  }
  
  for (int p = 0; p < (int)input_point_clouds.size(); ++p) {
    const PointCloud& input_point_cloud = input_point_clouds[p];
    const Panorama& panorama = panoramas[p];
    const vector<RasterizedGeometry>& rasterized_geometry = rasterized_geometries[p];

    const int width  = panorama.DepthWidth();
    const int height = panorama.DepthHeight();

    for (int q = 0; q < (int)input_point_cloud.GetNumPoints(); ++q) {
      const Point& point = input_point_cloud.GetPoint(q);
      const Vector2d pixel = panorama.ProjectToDepth(point.position);
      const int u = max(0, min(width - 1, static_cast<int>(round(pixel[0]))));
      const int v = max(0, min(height - 1, static_cast<int>(round(pixel[1]))));
      const int index = v * width + u;

      // No rasterized geometry. Very unlikely...
      if (rasterized_geometry[index].depth == initial_value.depth) {
        // cerr << "Rendering hole. This should rarely happen." << endl;
        continue;
      }
      
      const double depth_error = 
        fabs(rasterized_geometry[index].depth - (panorama.GetCenter() - point.position).norm()) / depth_unit;
      const double normal_error =
        acos(min(1.0, max(-1.0, rasterized_geometry[index].normal.dot(point.normal)))) * 180.0 / M_PI;

      errors[p][rasterized_geometry[index].geometry_type].first[0] += depth_error;
      errors[p][rasterized_geometry[index].geometry_type].first[1] += normal_error;
      errors[p][rasterized_geometry[index].geometry_type].second += 1;
    }
  }

  cerr << "=========================== Error ============================" << endl;
  for (int p = 0; p < panoramas.size(); ++p) {
    Vector2d error_sum(0, 0);
    int count = 0;
    for (const auto& type : all_geometry_types) {
      error_sum += errors[p][type].first;
      count     += errors[p][type].second;
    }
    cout << p << " : " << error_sum[0] / count << ' ' << error_sum[1] / count << endl;
  }
}
  
}  // namespace structured_indoor_modeling
