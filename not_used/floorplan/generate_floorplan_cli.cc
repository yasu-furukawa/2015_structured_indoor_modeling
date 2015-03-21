#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <gflags/gflags.h>

#include "../base/file_io.h"
#include "../base/floorplan.h"

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

void PrintRoom(const Floorplan& floorplan, const int room, ofstream& ofstr) {
  ofstr << "<polygon points=\"";
  for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
    ofstr << floorplan.GetRoomVertexLocal(room, v)[0] << ','
          << floorplan.GetRoomVertexLocal(room, v)[1] << ' ';
  }

  const string fill_color = "rgb(225,225,225)";
  const string line_color = "purple";
  
  const int kFirst = 0;
  ofstr << floorplan.GetRoomVertexLocal(room, kFirst)[0] << ','
        << floorplan.GetRoomVertexLocal(room, kFirst)[1] << ' ';
  ofstr << "\" style=\"fill:" << fill_color << ";stroke:" << line_color << ";stroke-width:100\" />" << endl;
}

void PrintDoor(const Floorplan& floorplan, const int door, ofstream& ofstr) {
  ofstr << "<polygon points=\"";

  Vector3d v3_0 = 
    floorplan.GetFloorplanToGlobal().transpose() * floorplan.GetDoorVertexGlobal(door, 0);
  Vector3d v3_1 = 
    floorplan.GetFloorplanToGlobal().transpose() * floorplan.GetDoorVertexGlobal(door, 1);
  Vector3d v3_4 = 
    floorplan.GetFloorplanToGlobal().transpose() * floorplan.GetDoorVertexGlobal(door, 4);
  Vector3d v3_5 = 
    floorplan.GetFloorplanToGlobal().transpose() * floorplan.GetDoorVertexGlobal(door, 5);

  Vector2d v0(v3_0[0], v3_0[1]);
  Vector2d v1(v3_1[0], v3_1[1]);
  Vector2d v4(v3_4[0], v3_4[1]);
  Vector2d v5(v3_5[0], v3_5[1]);

  Vector2d v01 = (v1 - v0).normalized();
  Vector2d v54 = (v4 - v5).normalized();

  // Position from v0.
  const double pos1 = (v1 - v0).dot(v01);
  const double pos4 = (v4 - v0).dot(v01);
  const double pos5 = (v5 - v0).dot(v01);

  // Move 4 to 1 and 5 to 0.
  v5 += (0.0 - pos5) * v54;
  v4 += (pos1 - pos4) * v54;

  const string fill_color = "rgb(125,125,125)";
  const string line_color = "purple";
  
  ofstr << v0[0] << ',' << v0[1] << ' '
        << v1[0] << ',' << v1[1] << ' '
        << v4[0] << ',' << v4[1] << ' '
        << v5[0] << ',' << v5[1] << ' '
        << "\" style=\"fill:" << fill_color << ";stroke:"
        << fill_color << ";stroke-width:100\" />" << endl;

  ofstr << "<line stroke-linecap=\"square\" x1=\"" << v0[0] << "\" y1=\"" << v0[1]
        << "\" x2=\"" << v5[0] << "\" y2=\"" << v5[1]
        << "\" style=\"stroke:" << line_color << ";stroke-width:100\" />" << endl;
  ofstr << "<line stroke-linecap=\"square\" x1=\"" << v1[0] << "\" y1=\"" << v1[1]
        << "\" x2=\"" << v4[0] << "\" y2=\"" << v4[1]
        << "\" style=\"stroke:" << line_color << ";stroke-width:100\" />" << endl;
}

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

  Floorplan floorplan;
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    //ifstr.open(file_io.GetFloorplanFinal().c_str());
    ifstr >> floorplan;
    ifstr.close();
  }

  {
    ofstream ofstr;
    ofstr.open(file_io.GetFloorplanSVG().c_str());
    ofstr << "<svg>" << endl;

    // Rooms.
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      PrintRoom(floorplan, room, ofstr);
    }

    // Doors.
    for (int door = 0; door < floorplan.GetNumDoors(); ++door) {
      PrintDoor(floorplan, door, ofstr);
    }
    
    ofstr << "</svg>" << endl;
    ofstr.close();
  }

  return 0;
}
