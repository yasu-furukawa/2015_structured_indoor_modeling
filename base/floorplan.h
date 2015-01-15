#ifndef FLOORPLAN_H__
#define FLOORPLAN_H__

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "floorplan_internal.h"

namespace structured_indoor_modeling {

class Floorplan {
 public:
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
  Triangle& GetFloorTriangle(const int room, const int triangle);
  Triangle& GetCeilingTriangle(const int room, const int triangle);
  Triangle& GetWallTriangle(const int room, const int wall, const int triangle);
  Triangle& GetDoorTriangle(const int door, const int triangle);
  Eigen::Vector3i GetGridSize() const;
  double GetGridUnit() const;

  double GetFloorHeight(const int room) const;
  double GetCeilingHeight(const int room) const;
  
  const std::vector<std::string>& GetRoomName(const int room) const;

  //----------------------------------------------------------------------
  Eigen::Vector2d LocalToGrid(const Eigen::Vector2d& local) const;
  Eigen::Vector2i LocalToGridInt(const Eigen::Vector2d& local) const;
  Eigen::Vector2d GridToLocal(const Eigen::Vector2d& grid) const;
  
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

  const WallTriangulation& GetWallTriangulation(const int room, const int wall) const;
  const FloorCeilingTriangulation& GetFloorTriangulation(const int room) const;
  const FloorCeilingTriangulation& GetCeilingTriangulation(const int room) const;

  WallTriangulation& GetWallTriangulation(const int room, const int wall);
  FloorCeilingTriangulation& GetFloorTriangulation(const int room);
  FloorCeilingTriangulation& GetCeilingTriangulation(const int room);
  
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

  void SetGrid(const int max_horizontal_size);

  friend std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
  friend std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);
};

std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
std::ostream& operator<<(std::ostream& ostr, const Floorplan& floorplan);

}  // namespace structured_indoor_modeling
 
#endif  // FLOORPLAN_H__
