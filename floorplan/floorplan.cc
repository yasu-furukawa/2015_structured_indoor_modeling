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

istream& operator>>(std::istream& istr, LineRoom& line_room) {
  int num_words;
  istr >> num_words;
  if (num_words != 0) {
    line_room.name.resize(num_words);
    for (int w = 0; w < num_words; ++w)
      istr >> line_room.name[w];
  }
  int num_points;
  istr >> num_points;
  
  line_room.contour.resize(num_points);
  for (int p = 0; p < num_points; ++p) {
    istr >> line_room.contour[p][0] >> line_room.contour[p][1];
  }
  
  istr >> line_room.floor_height
       >> line_room.ceiling_height;

  wall_triangulations.resize(num_points);
  for (int w = 0; w < num_points; ++w)
    istr >> wall_triangulations[w];

  istr >> floor_triangulation;
  istr >> ceiling_triangulation;
  return istr;
}

std::istream& operator>>(std::istream& istr, WallTriangulation& wall_triangulation) {
  int num_vertices, num_triangles;
  istr >> num_vertices >> num_triangles;
  wall_triangulation.vertices_in_uv.resize(num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    istr >> wall_triangulation.vertices_in_uv[v];
  }
  wall_triangulation.triangles.resize(num_triangles);
  for (int t = 0; t < num_triangles; ++t) {
    istr >> wall_triangulation.triangles[t];
  }

  return istr;
}

std::istream& operator>>(std::istream& istr, FloorCeilingTriangulation& triangulation) {


  return istr;
}

}  // namespace

istream& operator>>(istream& istr, LineFloorplan& line_floorplan) {
  for (int y = 0; y < 3; ++y)
    for (int x = 0; x < 3; ++x)
      istr >> line_floorplan.floorplan_to_global(y, x);
  
  int num_rooms;
  istr >> num_rooms;
  line_floorplan.line_rooms.resize(num_rooms);

  for (int r = 0; r < num_rooms; ++r)
    istr >> line_floorplan.line_rooms[r];

  int num_doors;
  istr >> num_doors;
  line_floorplan.line_doors.resize(num_doors);


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

