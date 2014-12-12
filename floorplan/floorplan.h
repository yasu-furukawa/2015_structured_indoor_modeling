#ifndef FLOORPLAN_H__
#define FLOORPLAN_H__

#include <Eigen/Dense>
#include <string>
#include <vector>

//----------------------------------------------------------------------
// Input data.
//----------------------------------------------------------------------
struct LineRoom {
  std::vector<std::string> name;
  std::vector<Eigen::Vector2d> walls;
  double floor_height;
  double ceiling_height;
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
