/*
  This file gives an access to reconstructed room, wall, floor,
  ceiling, door, and more. The data structure is complicated and one
  needs to first understand three different coordinate systems.

  1. Global coordinate system: Once all the 3D points are aligned,
  they live in this global coordinate system. Usually, Z axis points
  the gravity direction, but is not guaranteed.

  2. Local coordinate system: A building usually has three dominant
  (Manhattan) directions. A rotation is applied to the global
  coordinate system. In the local coordinate system, one can analyze
  building and room structure from a top down view with X and Y
  coordinates.

  3. Grid coordinate system: It is convenient to align a voxel grid to
  the local coordinate system, so that one can accumulate information
  per voxel. In short, a bounding box of the 3D points is
  computed. Then, X, Y, and Z coordinates are rescaled and translated
  independently. The resolution of the voxel grid is specified inside
  operator>> function in floorplan.cc. This number is the number of
  voxels along either X or Y axes (excluding Z), as one often analyzes
  building structure on the 2D X/Y plane.


  This class mostly offers accessors and implementation details are
  hidden in floorplan.cc. One should first use GetNumX function calls
  to know the number of elements. Then, do a for-loop to access their
  information. For the coordinate, the function name indicates the
  coordinate system to be used.


  The room shape is given as an array of 2D coordinates (XY) in the
  local coordinate frame. Therefore, the number of vertices for a
  room, and the number of walls for a room are equal. The wall may
  have doors (holes) and its face is fully represented by a set of
  triangles. Each wall has its own array of vertices with 3D
  coordinates, because we may need to use 3D vertices at the door
  corners. Similarly, the floor/celing face is represented by a set
  of triangles. However, for the floor/ceiling, the set of vertices in
  the triangulation are the same as the room vertices. Therefore, the
  floor/ceiling triangle simply stores the indexes to the room
  vertices.


  < Example >
  ifstream ifstr;
  ifstr.open("floorplan.txt");
  Floorplan floorplan;
  ifstr >> floorplan;
  ifstr.close();

  cout << floorplan.GetNumRooms() << " rooms." << endl;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    cout << "Room " << room << " has "
         << floorplan.GetNumRoomVertices() << " vertices." << endl;
  }
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
     for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(); ++vertex) {
        cout << floorplan.GetRoomVertexLocal(room, vertex) << endl;
     }
  }
*/

#ifndef FLOORPLAN_H__
#define FLOORPLAN_H__

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace structured_indoor_modeling {

struct Triangle;
struct WallTriangulation;
struct FloorCeilingTriangulation;
struct LineDoorFace;
struct LineDoor;
struct LineRoom;
  
class Floorplan {
 public:
  Floorplan();
  Floorplan(const std::string& filename);
  //----------------------------------------------------------------------
  // Accessors.
  //----------------------------------------------------------------------
  // GetNumX.
  int GetNumRooms() const;
  int GetNumRoomVertices(const int room) const;
  int GetNumWalls(const int room) const;
  int GetNumWallVertices(const int room, const int wall) const;
  int GetNumDoors() const;
  int GetNumDoorVertices(const int door) const;
  int GetNumDoorTriangles(const int door) const;
  
  int GetNumWallTriangles(const int room, const int wall) const;
  int GetNumFloorTriangles(const int room) const;
  int GetNumCeilingTriangles(const int room) const;

  //----------------------------------------------------------------------
  // 2D or 3D coordinate accessors.
  Eigen::Vector2d GetRoomVertexLocal(const int room, const int vertex) const;
  Eigen::Vector3d GetFloorVertexGlobal(const int room, const int vertex) const;
  Eigen::Vector3d GetCeilingVertexGlobal(const int room, const int vertex) const;
  Eigen::Vector3d GetWallVertexGlobal(const int room, const int wall, const int vertex) const;
  Eigen::Vector3d GetDoorVertexGlobal(const int door, const int vertex) const;

  //----------------------------------------------------------------------
  // Triangle information for the faces.  
  const Triangle& GetFloorTriangle(const int room, const int triangle) const;
  const Triangle& GetCeilingTriangle(const int room, const int triangle) const;
  const Triangle& GetWallTriangle(const int room, const int wall, const int triangle) const;
  const Triangle& GetDoorTriangle(const int door, const int triangle) const;
  Triangle& GetFloorTriangle(const int room, const int triangle);
  Triangle& GetCeilingTriangle(const int room, const int triangle);
  Triangle& GetWallTriangle(const int room, const int wall, const int triangle);
  Triangle& GetDoorTriangle(const int door, const int triangle);

  //----------------------------------------------------------------------
  // room/floor/ceiling/wall has 2D information along X and Y. Only
  // the floor and ceiling heights are given along Z direction for each
  // room. One exception is the door information, which has full X/Y/Z
  // information. Additional wall detail information (not implemented)
  // will also have full 3D information.
  double GetFloorHeight(const int room) const;
  double GetCeilingHeight(const int room) const;

  //----------------------------------------------------------------------
  // From scene recognition, a room name can be retrieved.  
  const std::vector<std::string>& GetRoomName(const int room) const;

  //----------------------------------------------------------------------
  // Conversion between the local and grid coordinate systems.
  // 2D usage is the main, and the 3D version is not currently
  // provided. GetGridSize gives the number of voxels along X/Y/Z axes.
  Eigen::Vector3i GetGridSize() const;
  double GetGridUnit() const;
  Eigen::Vector2d LocalToGrid(const Eigen::Vector2d& local) const;
  Eigen::Vector2i LocalToGridInt(const Eigen::Vector2d& local) const;
  Eigen::Vector2d GridToLocal(const Eigen::Vector2d& grid) const;

  //----------------------------------------------------------------------
  // If one needs to access the full triangulation information, use
  // this and look at the triangulation class definitions below.
  const WallTriangulation& GetWallTriangulation(const int room, const int wall) const;
  const FloorCeilingTriangulation& GetFloorTriangulation(const int room) const;
  const FloorCeilingTriangulation& GetCeilingTriangulation(const int room) const;

  WallTriangulation& GetWallTriangulation(const int room, const int wall);
  FloorCeilingTriangulation& GetFloorTriangulation(const int room);
  FloorCeilingTriangulation& GetCeilingTriangulation(const int room);

  //----------------------------------------------------------------------
  // If necessary, one can access the transformation from the local to
  // the global coordinate systems.
  const Eigen::Matrix3d& GetFloorplanToGlobal() const;

  // Room centers.
  Eigen::Vector2d GetRoomCenterLocal(const int room) const;
  Eigen::Vector3d GetRoomCenterGlobal(const int room) const;
  Eigen::Vector3d GetRoomCenterFloorGlobal(const int room) const;
  
 private:
  //----------------------------------------------------------------------
  // Transformation from floorplan to global.
  Eigen::Matrix3d floorplan_to_global;
  // Each room is an array of 2D coordinates.
  std::vector<LineRoom> line_rooms;
  // Door connections.
  std::vector<LineDoor> line_doors;

  // Grid information. Min/Max xyz coordinates plus some margin.
  Eigen::Vector2d grid_ranges[3];
  Eigen::Vector3i grid_size;
  double grid_unit;

  std::vector<Eigen::Vector2d> room_centers_local;
  
  void SetGrid(const int max_horizontal_size);

  friend std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
  friend std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);
};

std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);


//----------------------------------------------------------------------
// Data structure details. One should not really understand these to
// use this class.
//----------------------------------------------------------------------
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
 
#endif  // FLOORPLAN_H__
