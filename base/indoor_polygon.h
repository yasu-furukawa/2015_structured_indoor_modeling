#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

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
  // For type wall (room, wall).
  Eigen::Vector2i type_wall_info;
  // For type door (room0, room1).
  Eigen::Vector2i type_door_info;
  
  Normal normal;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Triangle> triangles;
};
    
class IndoorPolygon {
 public:
  IndoorPolygon();
  IndoorPolygon(const std::string& filename);

  int GetNumSegments();
  
 private:
  Eigen::Matrix4d floorplan_to_global;
  std::vector<Segment> segments;
};

std::istream& operator>>(std::istream& istr, Segment& segment);
std::ostream& operator<<(std::ostream& ostr, const Segment& segment);
std::istream& operator>>(std::istream& istr, IndoorPolygon& indoor_polygon);
std::ostream& operator<<(std::ostream& ostr, const IndoorPolygon& indoor_polygon);

}  // namespace structured_indoor_modeling
