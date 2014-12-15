#ifndef FLOORPLAN_H__
#define FLOORPLAN_H__

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "floorplan_internal.h"

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

  double GetFloorHeight(const int room) const;
  double GetCeilingHeight(const int room) const;

  const std::vector<std::string>& GetRoomName(const int room) const;
  
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

 private:
  //----------------------------------------------------------------------
  // Transformation from floorplan to global.
  Eigen::Matrix3d floorplan_to_global;
  // Each room is an array of 2D coordinates.
  std::vector<LineRoom> line_rooms;
  // Door connections.
  std::vector<LineDoor> line_doors;

  friend std::istream& operator>>(std::istream& istr, Floorplan& floorplan);
};

std::istream& operator>>(std::istream& istr, Floorplan& floorplan);

#endif  // FLOORPLAN_H__
