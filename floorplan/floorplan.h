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
struct LineRoom;

struct LineFloorplan {
  //----------------------------------------------------------------------
  // Accessors.
  //----------------------------------------------------------------------
  const Eigen::Matrix3d& GetFloorplanToGlobal() const;
  
  Eigen::Vector2d GetRoomVertexLocal(const int room, const int vertex) const;

  Eigen::Vector3d GetFloorVertexGlobal(const int room, const int vertex) const;
  Eigen::Vector3d GetCeilingVertexGlobal(const int room, const int vertex) const;
  Eigen::Vector3d GetWallVertexGlobal(const int room, const int wall, const int vertex) const;
  Eigen::Vector3d GetDoorVertexGlobal(const int door, const int vertex) const;
  
  const Triangle& GetFloorTriangle(const int room, const int triangle) const;
  const Triangle& GetCeilingTriangle(const int room, const int triangle) const;
  const Triangle& GetWallTriangle(const int room, const int wall, const int triangle) const;
  const Triangle& GetDoorTriangle(const int door, const int triangle) const;

  double GetFloorHeight(const int room) const;
  double GetCeilingHeight(const int room) const;
  
  int GetNumRooms() const;
  int GetNumRoomVertices(const int room) const;
  int GetNumWalls(const int room) const;
  int GetNumWallVertices(const int room, const int wall) const;
  int GetNumDoors();
  int GetNumDoorVertices(const int door);
  int GetNumDoorTriangles(const int door);
  
  int GetNumWallTriangles(const int room, const int wall) const;
  int GetNumFloorTriangles(const int room) const;
  int GetNumCeilingTriangles(const int room) const;

  //----------------------------------------------------------------------
  // Transformation from floorplan to global.
  Eigen::Matrix3d floorplan_to_global;
  // Each room is an array of 2D coordinates.
  std::vector<LineRoom> line_rooms;
  // Door connections.
  std::vector<LineDoor> line_doors;
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
  FloorCeilingTrianglulation ceiling_triangulation;
};

struct Triangle {
  // Vertex indices.
  Eigen::Vector3i indices;
  // Texture image index.
  int image_index;
  // UV coordinate.
  Vector2d uvs[3];
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
  Vector4_i vertex_indices;
};

struct LineDoor {
  LineDoorFace line_door_faces[2];
  Triangle triangles[8];
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
