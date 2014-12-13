#ifndef FLOORPLAN_H__
#define FLOORPLAN_H__

#include <Eigen/Dense>
#include <string>
#include <vector>

//----------------------------------------------------------------------
// Input data.
//----------------------------------------------------------------------
struct IUV;
struct Triangle;
struct WallTriangulation;
struct FloorCeilingTriangulation;

struct LineRoom {
  std::vector<std::string> name;

  // The following 3 information is enough to draw a simple 3D box for
  // a room.
  // Contour (2D coordinate) information.
  std::vector<Eigen::Vector2d> contour;
  double floor_height;
  double ceiling_height;

  // Wall triangulations.
  std::vector<WallTriangulation> wall_triangulations;
  // Floor triangulations.
  FloorCeilingTriangulation floor_triangulation;
  // Ceiling triangulations.
  FloorCeilingTrianglulation ceiling_triangulation;
};

// A texture coordinate in multi-texturing setup.
struct IUV {
  int index;
  double u;
  double v;
};

struct Triangle {
  // Vertex indices.
  Eigen::Vector3i indices;
  // Texture coordinate.
  IUV iuvs[3];
};

// Wall triangulation needs to store the vertex information.
struct WallTriangulation {
  // uv coordinates specify the coordinate.
  std::vector<Eigen::Vector2d> vertices_in_uv;
  std::vector<Triangle> triangles;
};

// Floor/ceiling triangulation needs not store the vertex information,
// because they are given by the room "contour".
struct FloorCeilingTriangulation {
  std::vector<Triangle> triangles;
};

struct LineDoorFace {
  int room_id;
  int wall_id;
  // Eigen::Vector2d position_ratio;
  Eigen::Vector2d left;
  Eigen::Vector2d right;
  double bottom_height;
  double top_height;
};  

typedef std::pair<LineDoorFace, LineDoorFace> LineDoor;

struct LineFloorplan {
  // Transformation from floorplan to global.
  Eigen::Matrix3d floorplan_to_global;
  // Each room is an array of 2D coordinates.
  std::vector<LineRoom> line_rooms;
  // Door connections.
  std::vector<LineDoor> line_doors;
};

//----------------------------------------------------------------------
// Output data.
//----------------------------------------------------------------------
struct Appearance {
  enum Type {
    Color,
    CrossTexture
  };

  Type type;
  Eigen::Vector3f color;
};

struct Face {
  Eigen::Vector2d positions[3];
};

struct RoomInterior {
  Appearance appearance;
  // A set of faces.
  std::vector<Face> faces;
};

struct Shape {
  std::vector<Eigen::Vector2d> vertices;
  std::vector<Eigen::Vector3i> faces;
};

// struct Door {
  // ??? To be determined
// };

struct FloorplanComponent {
  Shape outer_shape;
  std::vector<Shape> inner_shapes;
};

// Floorplan drawing consists of the following information.
// #0. Floor height.
// #1. Connected component.
// #3. Room interior (faces).
struct Floorplan {
  // #0.
  double floor_height;
  // #1.
  std::vector<FloorplanComponent> components;
  // #2.
  std::vector<RoomInterior> room_interiors;
};

std::istream& operator>>(std::istream& istr, LineFloorplan& line_floorplan);
std::istream& operator>>(std::istream& istr, Floorplan& floorplan);

std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);

#endif  // FLOORPLAN_H__
