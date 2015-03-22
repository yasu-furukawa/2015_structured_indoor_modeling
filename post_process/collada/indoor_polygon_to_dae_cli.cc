#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

#include <gflags/gflags.h>

#include "../../base/file_io.h"
#include "../../base/indoor_polygon.h"

DEFINE_string(floorplan_file, "", "Floorplan filename.");
DEFINE_string(indoor_polygon_file, "", "Indoor-polygon filename.");

using namespace Eigen;
using namespace structured_indoor_modeling;
using namespace std;

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
  
  string floorplan_file;
  if (FLAGS_floorplan_file == "") {
    floorplan_file = file_io.GetFloorplan();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", argv[1], FLAGS_floorplan_file.c_str());
    floorplan_file = buffer;
  }
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
