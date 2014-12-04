#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include "../calibration/file_io.h"
#include "floorplan_renderer.h"

using namespace std;

FloorplanRenderer::FloorplanRenderer() {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::RenderWireframe(const int room) {
  if (room < 0 ||
      static_cast<int>(line_floorplan.line_rooms.size()) <= room) {
    cerr << "Index out of bounds: " << room << endl;
    exit (1);
  }
  const LineRoom& line_room = line_floorplan.line_rooms[room];

  // Floor.
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glBegin(GL_LINES);
  glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
  for (int c = 0; c < static_cast<int>(line_room.walls.size()); ++c) {
    const int nextc = (c + 1) % (static_cast<int>(line_room.walls.size()));
    glColor4f(0.0f, 1.0f, 1.0f, 1.0f);

    Eigen::Vector3d floor0(line_room.walls[c][0],
                           line_room.walls[c][1],
                           line_room.floor_height);
    Eigen::Vector3d floor1(line_room.walls[nextc][0],
                           line_room.walls[nextc][1],
                           line_room.floor_height);
    Eigen::Vector3d ceiling0(line_room.walls[c][0],
                             line_room.walls[c][1],
                             line_room.ceiling_height);
    Eigen::Vector3d ceiling1(line_room.walls[nextc][0],
                             line_room.walls[nextc][1],
                             line_room.ceiling_height);
    
    floor0 = rotation * floor0;
    floor1 = rotation * floor1;
    ceiling0 = rotation * ceiling0;
    ceiling1 = rotation * ceiling1;

    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);

    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
    glVertex3f(ceiling1[0], ceiling1[1], ceiling1[2]);

    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
               
    /*
    glVertex3f(line_room.walls[c][0],
               line_room.walls[c][1],
               line_room.floor_height);
    glVertex3f(line_room.walls[nextc][0],
               line_room.walls[nextc][1],
               line_room.floor_height);
    */
  }
  glEnd();
  glEnable(GL_DEPTH_TEST);
}

void FloorplanRenderer::RenderWireframeAll() {
  for (int r = 0; r < static_cast<int>(line_floorplan.line_rooms.size()); ++r) {
    RenderWireframe(r);
  }
}

void FloorplanRenderer::Init(const string data_directory) {
  file_io::FileIO file_io(data_directory);
  
  ifstream ifstr;
  ifstr.open(file_io.GetFloorplan().c_str());
  ifstr >> line_floorplan;
  ifstr.close();

  {
    ifstream ifstr;
    ifstr.open(file_io.GetRotationMat().c_str());

    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        ifstr >> rotation(y, x);
        cout << rotation(y, x) << ' ';
      }
      cout << endl;
    }
    ifstr.close();
  }
}
/*
void FloorplanRenderer::SortWalls(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& direction,
                                  vector<Wall>* walls) {


}
*/

void FloorplanRenderer::RenderWallAll() {

}

void FloorplanRenderer::RenderWall(const int room) {

}
