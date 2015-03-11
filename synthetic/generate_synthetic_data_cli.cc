#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <string>
#include <gflags/gflags.h>

#include "../base/file_io.h"

using namespace Eigen;
using namespace std;

DEFINE_int32(width, 512, "Width of a depth panorama.");
DEFINE_int32(phi_range, 175.0 * M_PI / 180.0, "phi range.");

struct Point {
  Vector2i uv;
  Vector3d position;
  Vector3i color;
  Vector3d normal;
  int intensity;
};

struct DepthPixel {
  double depth;
  Vector3d normal;
};

struct Camera {
  Eigen::Vector3d center;
  int width;
  int height;
  
  double phi_range;
};

struct Mesh {
  vector<Vector3d> vertices;
  vector<Vector3i> triangles;
};

Eigen::Vector2d Project(const Camera& camera,
                        const Eigen::Vector3d& global) {  
  const Vector3d local = global - camera.center;
  const double phi_per_pixel = camera.phi_range / camera.height;
  // x coordinate.
  double theta = -atan2(local.y(), local.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * camera.width;
  const double depth = sqrt(local.x() * local.x() +
                            local.y() * local.y());
  double phi = atan2(local.z(), depth);
  const double pixel_offset_from_center = phi / phi_per_pixel;
  // uv[1] = height / 2.0 - pixel_offset_from_center;
  uv[1] = max(0.0, min(camera.height - 1.1,
                       camera.height / 2.0 -
                       pixel_offset_from_center));

  return uv;
}

Eigen::Vector3d Unproject(const Camera& camera,
                          const Eigen::Vector2d& pixel,
                          const double distance) {
  const double phi_per_pixel = camera.phi_range / camera.height;
  const double theta = -2.0 * M_PI * pixel[0] / camera.width;
  const double phi   = (camera.height / 2.0 - pixel[1]) * phi_per_pixel;

  Vector3d local;
  local[2] = distance * sin(phi);
  local[0] = distance * cos(phi) * cos(theta);
  local[1] = distance * cos(phi) * sin(theta);

  return local + camera.center;
}

void ReadCameras(const string& filename,
                 const int width,
                 const double phi_range,
                 vector<Camera>* cameras) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  if (!ifstr.is_open()) {
    cerr << "Cannot open a file." << filename << endl;
    exit (1);
  }
  while (true) {
    Camera camera;
    for (int i = 0; i < 3; ++i)
      ifstr >> camera.center[i];
    if (ifstr.eof())
      break;

    camera.width     = width;
    camera.height    = camera.width / 2;
    camera.phi_range = phi_range;
    cameras->push_back(camera);
  }
  ifstr.close();
}

void ReadMesh(const string& filename, Mesh * mesh) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  string stmp;
  for (int i = 0; i < 9; ++i)
    ifstr >> stmp;
  int num_of_vertices, num_of_faces;
  ifstr >> num_of_vertices;
  for (int i = 0; i < 11; ++i)
    ifstr >> stmp;
  ifstr >> num_of_faces;
  for (int i = 0; i < 6; ++i)
    ifstr >> stmp;

  mesh->vertices.resize(num_of_vertices);
  mesh->triangles.resize(num_of_faces);

  for (int v = 0; v < num_of_vertices; ++v) {
    for (int i = 0; i < 3; ++i)
      ifstr >> mesh->vertices[v][i];
  }

  for (int f = 0; f < num_of_faces; ++f) {
    int num;
    ifstr >> num;
    if (num != 3) {
      cerr << "Mesh must have only triangles." << endl;
      exit (1);
    }
    for (int i = 0; i < 3; ++i)
      ifstr >> mesh->triangles[f][i];
  }

  ifstr.close();
}

void WritePly(const string& filename, const vector<Point>& points) {
  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "ply" << endl
        << "format ascii 1.0" << endl
        << "comment created by MATLAB plywrite" << endl
        << "element vertex " << (int)points.size() << endl
        << "property int height" << endl
        << "property int width" << endl
        << "property float x" << endl
        << "property float y" << endl
        << "property float z" << endl
        << "property uchar red" << endl
        << "property uchar green" << endl
        << "property uchar blue" << endl
        << "property float nx" << endl
        << "property float ny" << endl
        << "property float nz" << endl
        << "property uchar intensity" << endl
        << "end_header" << endl;
  for (const auto& point : points) {
    ofstr << point.uv[0] << ' ' << point.uv[1] << ' '
          << point.position[0] << ' ' << point.position[1] << ' ' << point.position[2] << ' '
          << point.color[0] << ' ' << point.color[1] << ' ' << point.color[2] << ' '
          << point.normal[0] << ' ' << point.normal[1] << ' ' << point.normal[2] << ' '
          << point.intensity << endl;
  }
  ofstr.close();
}

double ComputeUnit(const Camera& camera,
                   const Vector3d& point) {
  const Vector2d uv = Project(camera, point);
  const double distance = (camera.center - point).norm();
  const Vector2d right(uv[0] + 1.0, uv[1]);
  const Vector3d right_point = Unproject(camera, right, distance);
  
  return (point - right_point).norm();  
}

void AddPointsFromDepths(const Camera& camera,
                         const vector<DepthPixel>& depths,
                         const double invalid_depth,
                         vector<Point>* points) {
  int index = 0;
  for (int y = 0; y < camera.height; ++y) {
    for (int x = 0; x < camera.width; ++x, ++index) {
      Point point;
      point.uv = Vector2i(x + 1, y + 1);
      if (depths[index].depth == invalid_depth) {
        point.position = Vector3d(0, 0, 0);
        point.color = Vector3i(0, 0, 0);
        point.normal = Vector3d(0, 0, 0);
        point.intensity = 0;
      } else {
        point.position = Unproject(camera, Vector2d(x, y), depths[index].depth);
        point.color = Vector3i(255, 255, 255);
        point.normal = depths[index].normal;
        point.intensity = 255;
      }
      points->push_back(point);
    }
  }
}

void Rasterize(const Camera& camera,
               const vector<Mesh>& meshes,
               vector<Point>* points) {
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

  // From depths to points.
  AddPointsFromDepths(camera, depths, kInvalidDepth, points);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    exit (1);
  }

  vector<Camera> cameras;
  {
    char buffer[1024];
    sprintf(buffer, "%s/pano_centers.txt", argv[1]);
    ReadCameras(buffer, FLAGS_width, FLAGS_phi_range, &cameras);
  }

  // Read a 3D model.
  vector<Mesh> meshes;
  {
    int index = 0;
    while (true) {
      char buffer[1024];
      sprintf(buffer, "%s/model-%03d.ply", argv[1], index);
      ifstream ifstr;
      ifstr.open(buffer);
      if (!ifstr.is_open())
        break;
      ifstr.close();
      
      Mesh mesh;
      ReadMesh(buffer, &mesh);
      meshes.push_back(mesh);
      ++index;
    }
  }

  // Start dumping out.
  for (int c = 0; c < cameras.size(); ++c) {
    cerr << "Camera: " << c << '/' << cameras.size() << endl;
    vector<Point> points;
    // Add center.
    Point center;
    center.uv = Vector2i(0, 0);
    center.position = cameras[c].center;
    center.color = Vector3i(0, 0, 0);
    center.normal = Vector3d(0, 0, 0);
    center.intensity = 0;
    points.push_back(center);
    // Rasterize.
    Rasterize(cameras[c], meshes, &points);

    char buffer[1024];
    sprintf(buffer, "%s/transformed_all/%03d.ply", argv[1], c);
    WritePly(buffer, points);
  }
  
  return 0;
}

  
