#include <fstream>
#include <iostream>
#include <numeric>
#include <string>

#include "floorplan.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {
  std::istream& operator>>(std::istream& istr, WallTriangulation& wall_triangulation);
  std::istream& operator>>(std::istream& istr, FloorCeilingTriangulation& triangulation);
  istream& operator>>(std::istream& istr, LineRoom& line_room);
  istream& operator>>(istream& istr, LineDoor& line_door);
  std::ostream& operator<<(std::ostream& ostr, const WallTriangulation& wall_triangulation);
  std::ostream& operator<<(std::ostream& ostr, const FloorCeilingTriangulation& triangulation);
  ostream& operator<<(std::ostream& ostr, const LineRoom& line_room);
  ostream& operator<<(ostream& ostr, const LineDoor& line_door);
}  // namespace;

Floorplan::Floorplan() {
}

Floorplan::Floorplan(const std::string& filename) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  ifstr >> *this;
  ifstr.close();

  const int num_room = GetNumRooms();
  room_centers_local.resize(num_room);
  for (int room = 0; room < num_room; ++room) {
    const int num_wall = GetNumWalls(room);
    room_centers_local[room] = Vector2d(0, 0);
    double denom = 0;
    for (int wall = 0; wall < num_wall; ++wall) {
      const int prev_wall = ((wall - 1) + num_wall) % num_wall;
      const int next_wall = (wall + 1) % num_wall;
      const double length =
        (GetRoomVertexLocal(room, wall) -
         GetRoomVertexLocal(room, prev_wall)).norm() +
        (GetRoomVertexLocal(room, wall) -
         GetRoomVertexLocal(room, next_wall)).norm();
      room_centers_local[room] += GetRoomVertexLocal(room, wall) * length;
      denom += length;
    }
    if (denom == 0.0) {
      cerr << "Impossible polygonRender." << endl;
      exit (1);
    }
    room_centers_local[room] /= denom;
  }
}
  
//----------------------------------------------------------------------
const Eigen::Matrix3d& Floorplan::GetFloorplanToGlobal() const {
  return floorplan_to_global;
}

Eigen::Vector2d Floorplan::GetRoomVertexLocal(const int room, const int vertex) const {
  return line_rooms[room].vertices[vertex];
}

Eigen::Vector3d Floorplan::GetFloorVertexGlobal(const int room, const int vertex) const {
  const Eigen::Vector2d local = GetRoomVertexLocal(room, vertex);
  return floorplan_to_global * Eigen::Vector3d(local[0], local[1], line_rooms[room].floor_height);
}

Eigen::Vector3d Floorplan::GetCeilingVertexGlobal(const int room, const int vertex) const {
  const Eigen::Vector2d local = GetRoomVertexLocal(room, vertex);
  return floorplan_to_global * Eigen::Vector3d(local[0], local[1], line_rooms[room].ceiling_height);
}

Eigen::Vector3d Floorplan::GetWallVertexGlobal(const int room, const int wall, const int vertex) const {
  const int v0 = wall;
  const int v1 = (wall + 1) % line_rooms[room].vertices.size();

  const Eigen::Vector3d v00 = GetFloorVertexGlobal(room, v0);
  const Eigen::Vector3d v01 = GetFloorVertexGlobal(room, v1);
  const Eigen::Vector3d v11 = GetCeilingVertexGlobal(room, v1);
  const Eigen::Vector3d v10 = GetCeilingVertexGlobal(room, v0);

  const Eigen::Vector2d& uv = line_rooms[room].wall_triangulations[wall].vertices_in_uv[vertex];
  // Must handle the 4 corner case exactly to avoid numerical errors.
  if (uv[0] == 0.0 && uv[1] == 0.0)
    return v00;
  else if (uv[0] == 0.0 && uv[1] == 1.0)
    return v10;
  else if (uv[0] == 1.0 && uv[1] == 0.0)
    return v01;
  else if (uv[0] == 1.0 && uv[1] == 1.0)
    return v11;
  else
    return v00 + (v01 - v00) * uv[0] + (v10 - v00) * uv[1];
}

Eigen::Vector3d Floorplan::GetDoorVertexGlobal(const int door, const int vertex) const {
  const int face = vertex / 4;
  const int face_vertex = vertex % 4;

  const int wall_vertex = line_doors[door].line_door_faces[face].vertex_indices[face_vertex];
  const int room = line_doors[door].line_door_faces[face].room_id;
  const int wall = line_doors[door].line_door_faces[face].wall_id;

  return GetWallVertexGlobal(room, wall, wall_vertex);
}

const Triangle& Floorplan::GetWallTriangle(const int room, const int wall, const int triangle) const {
  return line_rooms[room].wall_triangulations[wall].triangles[triangle];
}

const Triangle& Floorplan::GetFloorTriangle(const int room, const int triangle) const {
  return line_rooms[room].floor_triangulation.triangles[triangle];
}

const Triangle& Floorplan::GetCeilingTriangle(const int room, const int triangle) const {
  return line_rooms[room].ceiling_triangulation.triangles[triangle];
}

const Triangle& Floorplan::GetDoorTriangle(const int door, const int triangle) const {
  return line_doors[door].triangles[triangle];
}

Triangle& Floorplan::GetWallTriangle(const int room, const int wall, const int triangle) {
  return line_rooms[room].wall_triangulations[wall].triangles[triangle];
}

Triangle& Floorplan::GetFloorTriangle(const int room, const int triangle) {
  return line_rooms[room].floor_triangulation.triangles[triangle];
}

Triangle& Floorplan::GetCeilingTriangle(const int room, const int triangle) {
  return line_rooms[room].ceiling_triangulation.triangles[triangle];
}

Triangle& Floorplan::GetDoorTriangle(const int door, const int triangle) {
  return line_doors[door].triangles[triangle];
}

Eigen::Vector3i Floorplan::GetGridSize() const {
  return grid_size;
}

double Floorplan::GetGridUnit() const {
  return grid_unit;
}  

double Floorplan::GetFloorHeight(const int room) const {
  return line_rooms[room].floor_height;
}

double Floorplan::GetCeilingHeight(const int room) const {
  return line_rooms[room].ceiling_height;
}

const std::vector<std::string>& Floorplan::GetRoomName(const int room) const {
  return line_rooms[room].name;
}

int Floorplan::GetNumRooms() const {
  return static_cast<int>(line_rooms.size());
}
int Floorplan::GetNumRoomVertices(const int room) const {
  return static_cast<int>(line_rooms[room].vertices.size());
}
int Floorplan::GetNumWalls(const int room) const {
  return GetNumRoomVertices(room);
}
int Floorplan::GetNumWallVertices(const int room, const int wall) const {
  return static_cast<int>(line_rooms[room].wall_triangulations[wall].vertices_in_uv.size());
}
int Floorplan::GetNumDoors() const {
  return static_cast<int>(line_doors.size());
}
int Floorplan::GetNumDoorVertices(const int /*door*/) const {
  return 8;
}
int Floorplan::GetNumDoorTriangles(const int /*door*/) const {
  return 8;
}

int Floorplan::GetNumWallTriangles(const int room, const int wall) const {
  return static_cast<int>(line_rooms[room].wall_triangulations[wall].triangles.size());
}
int Floorplan::GetNumFloorTriangles(const int room) const {
  return static_cast<int>(line_rooms[room].floor_triangulation.triangles.size());
}
int Floorplan::GetNumCeilingTriangles(const int room) const {
  return static_cast<int>(line_rooms[room].ceiling_triangulation.triangles.size());
}

const WallTriangulation& Floorplan::GetWallTriangulation(const int room,
                                                         const int wall) const {
  return line_rooms[room].wall_triangulations[wall];
}

const FloorCeilingTriangulation& Floorplan::GetFloorTriangulation(const int room) const {
  return line_rooms[room].floor_triangulation;
}

const FloorCeilingTriangulation& Floorplan::GetCeilingTriangulation(const int room) const {
  return line_rooms[room].ceiling_triangulation;
}

WallTriangulation& Floorplan::GetWallTriangulation(const int room, const int wall) {
  return line_rooms[room].wall_triangulations[wall];
}

FloorCeilingTriangulation& Floorplan::GetFloorTriangulation(const int room) {
  return line_rooms[room].floor_triangulation;
}

FloorCeilingTriangulation& Floorplan::GetCeilingTriangulation(const int room) {
  return line_rooms[room].ceiling_triangulation;
}

//----------------------------------------------------------------------
void Floorplan::SetGrid(const int max_horizontal_size) {
  const int kMin = 0;
  const int kMax = 1;

  const int kX = 0;
  const int kY = 1;
  const int kZ = 2;
  
  for (int axis = 0; axis < 3; ++axis) {
    grid_ranges[axis][kMin] = numeric_limits<double>::max();
    grid_ranges[axis][kMax] = -numeric_limits<double>::max();
  }
  
  // First compute the min/max xyz.
  for (int room = 0; room < GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < GetNumRoomVertices(room); ++vertex) {
      const Vector2d local = GetRoomVertexLocal(room, vertex);
      for (int axis = 0; axis < 2; ++axis) {
        grid_ranges[axis][kMin] = min(grid_ranges[axis][kMin], local[axis]);
        grid_ranges[axis][kMax] = max(grid_ranges[axis][kMax], local[axis]);
      }
      grid_ranges[kZ][kMin] = min(grid_ranges[kZ][kMin], GetFloorHeight(room));
      grid_ranges[kZ][kMax] = max(grid_ranges[kZ][kMax], GetCeilingHeight(room));
    }      
  }

  //----------------------------------------------------------------------
  // Add 3 percent margin.
  const double kMarginRatio = 0.03;
  for (int axis = 0; axis < 3; ++axis) {
    const double length = grid_ranges[axis][kMax] - grid_ranges[axis][kMin];
    const double margin = kMarginRatio * length;
    grid_ranges[axis][kMax] += margin;
    grid_ranges[axis][kMin] -= margin;
  }

  const double max_length = max(grid_ranges[kX][kMax] - grid_ranges[kX][kMin],
                                grid_ranges[kY][kMax] - grid_ranges[kY][kMin]);
  grid_unit = max_length / max_horizontal_size;

  for (int axis = 0; axis < 3; ++axis) {
    grid_size[axis] =
      static_cast<int>(ceil((grid_ranges[axis][kMax] - grid_ranges[axis][kMin]) / grid_unit));
  }
}

Vector2d Floorplan::LocalToGrid(const Vector2d& local) const {
  Vector2d grid;
  for (int axis = 0; axis < 2; ++axis) {
    grid[axis] = (local[axis] - grid_ranges[axis][0]) / grid_unit;
  }
  return grid;
}

Vector2i Floorplan::LocalToGridInt(const Vector2d& local) const {
  Vector2d grid = LocalToGrid(local);
  Vector2i grid_int(max(0, min(grid_size[0] - 1, static_cast<int>(round(grid[0])))),
                    max(0, min(grid_size[1] - 1, static_cast<int>(round(grid[1])))));
  return grid_int;
}

Vector2d Floorplan::GridToLocal(const Vector2d& grid) const {
  Vector2d local;
  for (int axis = 0; axis < 2; ++axis) {
    local[axis] = grid[axis] * grid_unit + grid_ranges[axis][0];
  }
  return local;
}

Eigen::Vector2d Floorplan::GetRoomCenterLocal(const int room) const {
  return room_centers_local[room];
}

Eigen::Vector3d Floorplan::GetRoomCenterGlobal(const int room) const {
  const Eigen::Vector2d center = GetRoomCenterLocal(room);
  return floorplan_to_global *
    Eigen::Vector3d(center[0], center[1], (GetFloorHeight(room) + GetCeilingHeight(room)) / 2.0);
}

Eigen::Vector3d Floorplan::GetRoomCenterFloorGlobal(const int room) const {
  const Eigen::Vector2d center = GetRoomCenterLocal(room);
  return floorplan_to_global *
    Eigen::Vector3d(center[0], center[1], GetFloorHeight(room));
}
  
istream& operator>>(istream& istr, Floorplan& floorplan) {
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < 3; ++x)
      istr >> floorplan.floorplan_to_global(y, x);

  int num_rooms;
  istr >> num_rooms;
  floorplan.line_rooms.resize(num_rooms);

  for (int r = 0; r < num_rooms; ++r)
    istr >> floorplan.line_rooms[r];

  int num_doors;
  istr >> num_doors;

  floorplan.line_doors.resize(num_doors);
  for (int d = 0; d < num_doors; ++d) {
    istr >> floorplan.line_doors[d];
  }

  const int kMaxHorizontalSize = 1024;
  floorplan.SetGrid(kMaxHorizontalSize);
  
  return istr;
}

ostream& operator<<(ostream& ostr, const Floorplan& floorplan) {
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      ostr << floorplan.floorplan_to_global(y, x) << ' ';
    }
    ostr << endl;
  }

  ostr << floorplan.GetNumRooms() << endl;
  for (int r = 0; r < floorplan.GetNumRooms(); ++r)
    ostr << floorplan.line_rooms[r];

  ostr << floorplan.GetNumDoors() << endl;
  for (int d = 0; d < floorplan.GetNumDoors(); ++d) {
    ostr << floorplan.line_doors[d];
  }
  return ostr;
}


//======================================================================
// Local utility function.s
//======================================================================
namespace {
//----------------------------------------------------------------------
// Read.
//----------------------------------------------------------------------  
std::istream& operator>>(std::istream& istr, WallTriangulation& wall_triangulation) {
  int num_vertices, num_triangles;
  istr >> num_vertices >> num_triangles;
  wall_triangulation.vertices_in_uv.resize(num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    istr >> wall_triangulation.vertices_in_uv[v][0]
         >> wall_triangulation.vertices_in_uv[v][1];
  }
  wall_triangulation.triangles.resize(num_triangles);
  for (int t = 0; t < num_triangles; ++t) {
    istr >> wall_triangulation.triangles[t];
  }

  return istr;
}

std::istream& operator>>(std::istream& istr, FloorCeilingTriangulation& triangulation) {
  int num_triangles;
  istr >> num_triangles;
  triangulation.triangles.resize(num_triangles);
  for (int t = 0; t < num_triangles; ++t) {
    istr >> triangulation.triangles[t];
  }

  return istr;
}

istream& operator>>(std::istream& istr, LineRoom& line_room) {
  int num_words;
  istr >> num_words;
  if (num_words != 0) {
    line_room.name.resize(num_words);
    for (int w = 0; w < num_words; ++w)
      istr >> line_room.name[w];
  }
  int num_vertices;
  istr >> num_vertices;
  
  line_room.vertices.resize(num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    istr >> line_room.vertices[v][0] >> line_room.vertices[v][1];
  }
  
  istr >> line_room.floor_height
       >> line_room.ceiling_height;

  line_room.wall_triangulations.resize(num_vertices);
  for (int w = 0; w < num_vertices; ++w)
    istr >> line_room.wall_triangulations[w];

  istr >> line_room.floor_triangulation;
  istr >> line_room.ceiling_triangulation;
  return istr;
}
  
istream& operator>>(istream& istr, LineDoor& line_door) {
  for (int i = 0; i < 2; ++i) {
    istr >> line_door.line_door_faces[i].room_id
         >> line_door.line_door_faces[i].wall_id;
  }
  
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 4; ++j) {
      istr >> line_door.line_door_faces[i].vertex_indices[j];
    }
  }

  const int kNumTriangles = 8;
  for (int t = 0; t < kNumTriangles; ++t) {
    istr >> line_door.triangles[t];
  }
  
  return istr;
}

//----------------------------------------------------------------------
// Write.
//----------------------------------------------------------------------
std::ostream& operator<<(std::ostream& ostr, const WallTriangulation& wall_triangulation) {
  const int num_vertices = static_cast<int>(wall_triangulation.vertices_in_uv.size());
  const int num_triangles = static_cast<int>(wall_triangulation.triangles.size());
  ostr << num_vertices << ' ' << num_triangles << endl;

  for (int v = 0; v < num_vertices; ++v) {
    ostr << wall_triangulation.vertices_in_uv[v][0] << ' '
         << wall_triangulation.vertices_in_uv[v][1] << endl;
  }
  
  for (int t = 0; t < num_triangles; ++t) {
    ostr << wall_triangulation.triangles[t];
  }

  return ostr;
}

std::ostream& operator<<(std::ostream& ostr, const FloorCeilingTriangulation& triangulation) {
  const int num_triangles = static_cast<int>(triangulation.triangles.size());
  ostr << num_triangles << endl;
  for (int t = 0; t < num_triangles; ++t) {
    ostr << triangulation.triangles[t];
  }

  return ostr;
}

ostream& operator<<(std::ostream& ostr, const LineRoom& line_room) {
  const int num_words = line_room.name.size();
  ostr << static_cast<int>(num_words) << endl;

  if (num_words != 0) {
    for (int w = 0; w < num_words; ++w)
      ostr << line_room.name[w] << endl;
  }
  const int num_vertices = static_cast<int>(line_room.vertices.size());
  ostr << num_vertices << endl;
  
  for (int v = 0; v < num_vertices; ++v) {
    ostr << line_room.vertices[v][0] << ' ' << line_room.vertices[v][1] << endl;
  }
  
  ostr << line_room.floor_height << ' '
       << line_room.ceiling_height << endl;

  for (int w = 0; w < num_vertices; ++w)
    ostr << line_room.wall_triangulations[w];

  ostr << line_room.floor_triangulation << ' ' << line_room.ceiling_triangulation << endl;
  return ostr;
}
  
ostream& operator<<(ostream& ostr, const LineDoor& line_door) {
  for (int i = 0; i < 2; ++i) {
    ostr << line_door.line_door_faces[i].room_id << ' '
         << line_door.line_door_faces[i].wall_id << endl;
  }
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 4; ++j) {
      ostr << line_door.line_door_faces[i].vertex_indices[j] << ' ';
    }
    ostr << endl;
  }

  const int kNumTriangles = 8;
  for (int t = 0; t < kNumTriangles; ++t) {
    ostr << line_door.triangles[t] << endl;
  }
  
  return ostr;
}
  
}  // namespace
   
/*
istream& operator>>(istream& istr, Floorplan& floorplan) {
  string header;
  int component_num;
  istr >> header
       >> floorplan.floor_height
       >> component_num;

  floorplan.components.resize(component_num);
  for (auto& component : floorplan.components) {
    istr >> component.outer_shape;
    int inner_shape_num;
    istr >> inner_shape_num;
    component.inner_shapes.resize(inner_shape_num);
    for (auto& inner_shape : component.inner_shapes) {
      istr >> inner_shape;
    }
  }
  
  return istr;
}

ostream& operator<<(ostream& ostr, const Floorplan& floorplan) {
  ostr << "FLOORPLAN" << endl
       << floorplan.floor_height << endl
       << floorplan.components.size() << endl;
  // Components.
  for (const auto& component : floorplan.components) {
    // outer_shape.
    ostr << component.outer_shape;
    // inner_shapes.
    ostr << component.inner_shapes.size() << endl;
    for (const auto& shape : component.inner_shapes) {
      ostr << shape << endl;
    }
  }
  // Room interiors.
  
  return ostr;
}
*/

/*
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
*/


/*
istream& operator>>(istream& istr, Shape& shape) {
  int vnum;
  istr >> vnum;
  shape.vertices.resize(vnum);
  for (auto& vertex : shape.vertices) {
    istr >> vertex[0] >> vertex[1];
  }
  int fnum;
  istr >> fnum;
  shape.faces.resize(fnum);
  for (auto& face : shape.faces) {
    istr >> face[0] >> face[1] >> face[2];
  }
  return istr;
}

ostream& operator<<(ostream& ostr, const Shape& shape) {
  ostr << shape.vertices.size() << endl;
  for (const auto& vertex : shape.vertices) {
    ostr << vertex[0] << ' ' << vertex[1] << endl;
  }
  ostr << shape.faces.size() << endl;
  for (const auto& face : shape.faces) {
    ostr << face[0] << ' ' << face[1] << ' ' << face[2] << endl;
  }
  return ostr;
}

std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);

*/  

}  // namespace structured_indoor_modeling
  
