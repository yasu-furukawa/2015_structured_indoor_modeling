#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <gflags/gflags.h>

#include "../base/file_io.h"

using namespace Eigen;
using namespace std;

DEFINE_int32(width, 512, "Width of a depth panorama.");
DEFINE_int32(phi_range, 150.0 * M_PI / 180.0, "phi range.");

struct Point {
  Vector2d uv;
  Vector3d position;
  Vector3i color;
  Vector3d normal;
  int intensity;
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
  const double phi_per_pixel =
    camera.phi_range / camera.height;

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
    ifstr >> num_of_faces;

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

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[1] << " data_directory" << endl;
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

  
  
  return 0;
}

  
