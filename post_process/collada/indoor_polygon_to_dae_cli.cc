#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

#include <gflags/gflags.h>

#include "../../base/file_io.h"
#include "../../base/indoor_polygon.h"

using namespace Eigen;
using namespace structured_indoor_modeling;
using namespace std;

namespace {

struct Polygon {
  vector<int> boundary;
  vector<vector<int> > holes;
};

void SetPolygon(const Segment& segment, const int offset, Polygon* polygon) {
  set<pair<int, int> > edges;
  for (const auto& triangle : segment.triangles) {
    for (int lhs = 0; lhs < 3; ++lhs) {
      const int rhs = (lhs + 1) % 3;
      const pair<int, int> forward(triangle.indices[lhs], triangle.indices[rhs]);
      const pair<int, int> backward(triangle.indices[rhs], triangle.indices[lhs]);

      if (edges.find(backward) != edges.end()) {
        edges.erase(backward);
      } else {
        edges.insert(forward);
      }
    }
  }

  //----------------------------------------------------------------------
  // Form chains.
  vector<vector<int> > chains;
  while (!edges.empty()) {
    vector<int> chain;
    

    chains.push_back(chain);
  }
}
  
}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
#ifdef __APPLE__
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

  FileIO file_io(argv[1]);
  IndoorPolygon indoor_polygon(file_io.GetIndoorPolygon());

  vector<Vector3d> vertices;
  vector<Polygon> polygons;

  for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
    const Segment& segment = indoor_polygon.GetSegment(s);
    const int offset = vertices.size();
    for (const auto& vertex : segment.vertices)
      vertices.push_back(vertex);

    Polygon polygon;
    SetPolygon(segment, offset, &polygon);

    polygons.push_back(polygon);
  }
  
  ofstream ofstr;
  ofstr.open(file_io.GetCollada().c_str());

  ofstr << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << endl
        << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">" << endl
        << "<asset>" << endl
        << "<up_axis>Z_UP</up_axis>" << endl
        << "</asset>" << endl
        << "<library_visual_scenes>" << endl
        << "<visual_scene id=\"ID1\">" << endl
        << "<node name=\"SketchUp\">" << endl
        << "<instance_geometry url=\"#ID10\">" << endl
        << "</instance_geometry>" << endl
        << "</node>" << endl
        << "</visual_scene>" << endl
        << "</library_visual_scenes>" << endl
        << "<library_geometries>" << endl
        << "<geometry id=\"ID10\">" << endl
        << "<mesh>" << endl
        << "<source id=\"ID11\">" << endl
        << "<float_array id=\"ID14\" count=\"" << (int)vertices.size() * 3 << "\">";
  for (const auto& vertex : vertices) {
    ofstr << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << ' ';
  }
  ofstr << "</float_array>" << endl
        << "<technique_common>" << endl
        << "<accessor count=\"" << (int)vertices.size() << "\" source=\"#ID14\" stride=\"3\">" << endl
        << "<param name=\"X\" type=\"float\" />" << endl
        << "<param name=\"Y\" type=\"float\" />" << endl
        << "<param name=\"Z\" type=\"float\" />" << endl
        << "</accessor>" << endl
        << "</technique_common>" << endl
        << "</source>" << endl
        << "<vertices id=\"ID13\">" << endl
        << "<input semantic=\"POSITION\" source=\"#ID11\" />" << endl
        << "</vertices>" << endl
        << "<polygons count=\"" << (int)polygons.size() << "\" material=\"Material2\">" << endl
        << "<input offset=\"0\" semantic=\"VERTEX\" source=\"#ID13\" />" << endl;
  for (const auto& polygon : polygons) {
    if (polygon.holes.empty()) {
      ofstr << "<p>";
      for (const auto& index : polygon.boundary)
        ofstr << index << ' ';
      ofstr << "</p>" << endl;
    } else {
      ofstr << "<ph>" << endl
            << "<p>";
      for (const auto& index : polygon.boundary)
        ofstr << index << ' ';
      ofstr << "</p>" << endl;
      for (const auto& hole : polygon.holes) {
        ofstr << "<h>";
        for (const auto& index : hole)
          ofstr << index << ' ';
        ofstr << "</h>" << endl;
      }
      ofstr << "</ph>" << endl;
    }
  }
  ofstr << "</polygons>" << endl
        << "</mesh>" << endl
        << "</geometry>" << endl
        << "</library_geometries>" << endl
        << "<scene>" << endl
        << "<instance_visual_scene url=\"#ID1\" />" << endl
        << "</scene>" << endl
        << "</COLLADA>" << endl;
  
  ofstr.close();
  
  /*
  Floorplan floorplan(floorplan_file);

  // vcoords
  vector<Vector3d> vertices;
  vector<int> vcounts;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      vcounts.push_back(4);
      const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);

      {
        Vector2d current = floorplan.GetRoomVertexLocal(room, wall);
        Vector2d next = floorplan.GetRoomVertexLocal(room, next_wall);

        const double floor = floorplan.GetFloorHeight(room);
        const double ceiling = floorplan.GetCeilingHeight(room);
        vertices.push_back(Vector3d(current[0], current[1], floor));
        vertices.push_back(Vector3d(next[0], next[1], floor));
        vertices.push_back(Vector3d(next[0], next[1], ceiling));
        vertices.push_back(Vector3d(current[0], current[1], ceiling));
      }
    }

    vcounts.push_back(floorplan.GetNumWalls(room));
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      Vector2d current = floorplan.GetRoomVertexLocal(room, wall);
      const double floor = floorplan.GetFloorHeight(room);
      vertices.push_back(Vector3d(current[0], current[1], floor));
    }
  }  
  
  cout << "<source id=\"ID5\">" << endl
       << "<float_array id=\"ID8\" count=\"" << (int)vertices.size() * 3  << "\">";

  for (const auto& vertex : vertices)
    cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << ' ';
    
  cout << "</float_array>" << endl
       << "<technique_common>" << endl
       << "<accessor count=\"" << (int)vertices.size() << "\" source=\"#ID8\" stride=\"3\">" << endl
       << "<param name=\"X\" type=\"float\" />" << endl
       << "<param name=\"Y\" type=\"float\" />" << endl
       << "<param name=\"Z\" type=\"float\" />" << endl
       << "</accessor>" << endl
       << "</technique_common>" << endl
       << "</source>" << endl;

  //----------------------------------------------------------------------
  cout << endl << endl;
  //----------------------------------------------------------------------

  cout << "<polylist count=\"" << (int)vcounts.size() << "\" material=\"Material2\">" << endl
       << "<input offset=\"0\" semantic=\"VERTEX\" source=\"#ID7\" />" << endl
       << "<vcount>";

  for (const auto& vcount : vcounts)
    cout << vcount << ' ';
  cout << "</vcount>" << endl
       << "<p>";

  for (int i = 0; i < (int)vertices.size(); ++i)
    cout << i << ' ';

  cout << "</p>" << endl
       << "</polylist>" << endl;
  */
  return 0;
}
