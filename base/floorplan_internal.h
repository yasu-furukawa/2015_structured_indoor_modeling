#ifndef FLOORPLAN_INTERNAL_H_
#define FLOORPLAN_INTERNAL_H_

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace structured_indoor_modeling {

struct Triangle {
  // Vertex indices.
  Eigen::Vector3i indices;
  // Texture image index.
  int image_index;
  // UV coordinate.
  Eigen::Vector2d uvs[3];
};

// Wall triangulation needs to store the vertex information.
struct WallTriangulation {
  // uv coordinates specify the coordinate.
  std::vector<Eigen::Vector2d> vertices_in_uv;
  std::vector<Triangle> triangles;
};

// Floor/ceiling triangulation needs not store the vertex information,
// because they are given by the vertices.
struct FloorCeilingTriangulation {
  std::vector<Triangle> triangles;
};

struct LineDoorFace {
  int room_id;
  int wall_id;
  Eigen::Vector4i vertex_indices;
};

struct LineDoor {
  LineDoorFace line_door_faces[2];
  Triangle triangles[8];
};

struct LineRoom {
  std::vector<std::string> name;

  // The following 3 information is enough to draw a simple 3D box for
  // a room.
  // Vertices (2D coordinate) information.
  std::vector<Eigen::Vector2d> vertices;
  double floor_height;
  double ceiling_height;

  // Wall triangulations.
  std::vector<WallTriangulation> wall_triangulations;
  // Floor triangulations.
  FloorCeilingTriangulation floor_triangulation;
  // Ceiling triangulations.
  FloorCeilingTriangulation ceiling_triangulation;
};

}  // namespace structured_indoor_modeling
 
#endif  // FLOORPLAN_INTERNAL_H_
