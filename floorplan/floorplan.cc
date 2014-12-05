#include <iostream>
#include <string>

#include "floorplan.h"

using namespace Eigen;
using namespace std;

namespace {
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

}  // namespace

istream& operator>>(istream& istr, LineFloorplan& line_floorplan) {
  int num_rooms, num_doors;
  istr >> num_rooms >> num_doors;
  line_floorplan.line_rooms.resize(num_rooms);
  line_floorplan.line_doors.resize(num_doors);

  for (int r = 0; r < num_rooms; ++r) {
    LineRoom& line_room = line_floorplan.line_rooms[r];
    
    int room_index;
    int num_walls;
    istr >> room_index
         >> num_walls
         >> line_room.floor_height
         >> line_room.ceiling_height;
    
    line_room.walls.resize(num_walls);
    
    for (int w = 0; w < num_walls; ++w) {
      double dummy;
      istr >> dummy >> dummy >> line_room.walls[w][0] >> line_room.walls[w][1];
    }
  }

  for (int d = 0; d < num_doors; ++d) {
    LineDoor& line_door = line_floorplan.line_doors[d];
    
    int door_index;
    double dummy;
    istr >> door_index;
    istr >> line_door.first.room_id >> line_door.first.wall_id
         >> line_door.first.left[0] >> line_door.first.left[1] >> line_door.first.bottom_height
         >> line_door.first.right[0] >> line_door.first.right[1] >> line_door.first.top_height;
    for (int i = 0; i < 6; ++i)
      istr >> dummy;

    istr >> line_door.second.room_id >> line_door.second.wall_id
         >> line_door.second.left[0] >> line_door.second.left[1] >> line_door.second.bottom_height
         >> line_door.second.right[0] >> line_door.second.right[1] >> line_door.second.top_height;
    for (int i = 0; i < 6; ++i)
      istr >> dummy;
  }
  
  return istr;
}
  
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

