#include <fstream>
#include <iostream>
#include "indoor_polygon.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

IndoorPolygon::IndoorPolygon() {
}

IndoorPolygon::IndoorPolygon(const std::string& filename) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  ifstr >> *this;
  ifstr.close();
}
  
Eigen::Vector3d IndoorPolygon::ManhattanToGlobal(const Eigen::Vector3d& manhattan) const {
  Eigen::Vector4d coord(manhattan[0],
                        manhattan[1],
                        manhattan[2],
                        1.0);
  coord = manhattan_to_global * coord;
  return Eigen::Vector3d(coord[0], coord[1], coord[2]);
}

Eigen::Vector3d IndoorPolygon::GlobalToManhattan(const Eigen::Vector3d& global) const {
  Eigen::Vector4d coord(global[0],
                        global[1],
                        global[2],
                        1.0);
  coord = global_to_manhattan * coord;
  return Eigen::Vector3d(coord[0], coord[1], coord[2]);
}  

std::istream& operator>>(std::istream& istr, Segment& segment) {
  string header;
  istr >> header;
  {
    istr >> header;
    if (header == "floor") {
      segment.type = Segment::FLOOR;
      istr >> segment.floor_info;
    } else if (header == "ceiling") {
      segment.type = Segment::CEILING;
      /*
    } else if (header == "wall") {
      segment.type = Segment::WALL;
      istr >> segment.type_wall_info[0] >> segment.type_wall_info[1];
      */
    } else if (header == "room") {
      segment.type = Segment::WALL;
      istr >> segment.type_wall_info[0] >> header >> segment.type_wall_info[1];
    } else if (header == "door") {
      segment.type = Segment::DOOR;
      istr >> segment.type_door_info[0] >> segment.type_door_info[1];
    } else {
      cerr << "Invalid segment type: " << header << endl;
      exit (1);
    }
  }

  {
    istr >> header;
    if (header == "X") {
      segment.normal = Segment::PositiveX;
    } else if (header == "-X") {
      segment.normal = Segment::NegativeX;
    } else if (header == "Y") {
      segment.normal = Segment::PositiveY;
    } else if (header == "-Y") {
      segment.normal = Segment::NegativeY;
    } else if (header == "Z") {
      segment.normal = Segment::PositiveZ;
    } else if (header == "-Z") {
      segment.normal = Segment::NegativeZ;
    } else {
      cerr << "Invalid normal: " << header << endl;
      exit (1);
    }
  }

  int num_vertices, num_triangles;
  istr >> num_vertices >> num_triangles;
  {  
    segment.vertices.clear();
    segment.vertices.resize(num_vertices);    
    for (int v = 0; v < num_vertices; ++v) {
      istr >> segment.vertices[v][0]
           >> segment.vertices[v][1]
           >> segment.vertices[v][2];
    }
  }

  {
    segment.triangles.clear();
    segment.triangles.resize(num_triangles);
    for (int t = 0; t < num_triangles; ++t) {
      istr >> segment.triangles[t];
    }
  }

  return istr;
}
  
std::ostream& operator<<(std::ostream& ostr, const Segment& segment) {
  ostr << "SEGMENT" << endl;
  switch (segment.type) {
  case Segment::FLOOR: {
    ostr << "floor " << segment.floor_info << endl;
    break;
  }
  case Segment::CEILING: {
    ostr << "ceiling" << endl;
    break;
  }
    /*
  case Segment::WALL: {
    ostr << "wall " << segment.type_wall_info[0] << ' ' << segment.type_wall_info[1] << endl;
    break;
  }
    */
  case Segment::WALL: {
    ostr << "room " << segment.type_wall_info[0] << " wall " << segment.type_wall_info[1] << endl;
    break;
  }
  case Segment::DOOR: {
    ostr << "door " << segment.type_door_info[0] << ' ' << segment.type_door_info[1] << endl;
    break;
  }
  default: {
    cerr << "Invalid segment type." << endl;
    exit (1);
  }
  }

  switch (segment.normal) {
  case Segment::PositiveX: {
    ostr << "X" << endl;
    break;
  }
  case Segment::NegativeX: {
    ostr << "-X" << endl;
    break;
  }
  case Segment::PositiveY: {
    ostr << "Y" << endl;
    break;
  }
  case Segment::NegativeY: {
    ostr << "-Y" << endl;
    break;
  }
  case Segment::PositiveZ: {
    ostr << "Z" << endl;
    break;
  }
  case Segment::NegativeZ: {
    ostr << "-Z" << endl;
    break;
  }
  default: {
    cerr << "Invalid segment normal." << endl;
    exit (1);
  }
  }

  ostr << static_cast<int>(segment.vertices.size()) << ' '
       << static_cast<int>(segment.triangles.size()) << endl;

  for (const auto& vertex : segment.vertices) {
    ostr << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << endl;
  }

  for (const auto& triangle : segment.triangles) {
    ostr << triangle << endl;
  }
      
  return ostr;
}
  
std::istream& operator>>(std::istream& istr, IndoorPolygon& indoor_polygon) {
  string header;
  istr >> header;
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      istr >> indoor_polygon.manhattan_to_global(y, x);
    }
  }

  {
    Matrix3d rotation = indoor_polygon.manhattan_to_global.block(0, 0, 3, 3);

    indoor_polygon.global_to_manhattan.block(0, 0, 3, 3) = rotation.transpose();
    indoor_polygon.global_to_manhattan.block(0, 3, 3, 1) =
      - rotation.transpose() * indoor_polygon.manhattan_to_global.block(0, 3, 3, 1);
    indoor_polygon.global_to_manhattan(3, 0) = 0.0;
    indoor_polygon.global_to_manhattan(3, 1) = 0.0;
    indoor_polygon.global_to_manhattan(3, 2) = 0.0;
    indoor_polygon.global_to_manhattan(3, 3) = 1.0;
  }
  
  int num_segments;
  istr >> num_segments;
  indoor_polygon.segments.clear();
  indoor_polygon.segments.resize(num_segments);

  for (int s = 0; s < num_segments; ++s) {
    istr >> indoor_polygon.segments[s];
  }
  
  return istr;
}
  
std::ostream& operator<<(std::ostream& ostr, const IndoorPolygon& indoor_polygon) {
  ostr << "INDOOR_POLYGON" << endl;
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      ostr << indoor_polygon.manhattan_to_global(y, x) << ' ';
    }
    ostr << endl;
  }
  ostr << indoor_polygon.GetNumSegments() << endl;
  for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
    ostr << indoor_polygon.GetSegment(s) << endl;
  }
  
  return ostr;
}  

}  // namespace structured_indoor_modeling
