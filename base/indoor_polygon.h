#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include "geometry.h"

namespace structured_indoor_modeling {

struct Segment {
  enum Type {
    FLOOR,
    CEILING,
    WALL,
    DOOR
  };

  enum Normal {
    PositiveX,
    NegativeX,
    PositiveY,
    NegativeY,
    PositiveZ,
    NegativeZ
  };
  
  Type type;
  // Room number.
  int floor_info;
  int ceiling_info;
  // For wall (room, wall).
  Eigen::Vector2i wall_info;
  // For door (room0, wall0, room1, wall1).
  Eigen::Vector4i door_info;
  
  Normal normal;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Triangle> triangles;
};
    
class IndoorPolygon {
 public:
  IndoorPolygon();
  IndoorPolygon(const std::string& filename);

  void InitFromBinaryPly(const std::string& filename);

  int GetNumSegments() const { return segments.size(); }
  const Segment& GetSegment(const int segment) const { return segments[segment]; }
  Segment& GetSegment(const int segment) { return segments[segment]; }

  Eigen::Vector3d ManhattanToGlobal(const Eigen::Vector3d& manhattan) const;
  Eigen::Vector3d GlobalToManhattan(const Eigen::Vector3d& global) const;
  
 private:
  Eigen::Matrix4d manhattan_to_global;
  Eigen::Matrix4d global_to_manhattan;
  std::vector<Segment> segments;

  friend std::istream& operator>>(std::istream& istr, IndoorPolygon& indoor_polygon);
  friend std::ostream& operator<<(std::ostream& ostr, const IndoorPolygon& indoor_polygon);
};

std::istream& operator>>(std::istream& istr, Segment& segment);
std::ostream& operator<<(std::ostream& ostr, const Segment& segment);
std::istream& operator>>(std::istream& istr, IndoorPolygon& indoor_polygon);
std::ostream& operator<<(std::ostream& ostr, const IndoorPolygon& indoor_polygon);

}  // namespace structured_indoor_modeling
