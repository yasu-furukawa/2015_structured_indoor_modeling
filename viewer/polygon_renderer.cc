#include <fstream>
#include <iostream>
#include <Eigen/Dense>

#include "../calibration/file_io.h"
#include "polygon_renderer.h"

using namespace std;

Eigen::Vector2d center_for_wall_comparer;

struct Wall {
  Eigen::Vector2d points[2];
};

typedef vector<Wall> Walls;

namespace {
bool WallComparer(const Wall& lhs, const Wall& rhs) {
  Wall lhs_diff, rhs_diff;
  for (int i = 0; i < 2; ++i) {
    lhs_diff.points[i] = lhs.points[i] - center_for_wall_comparer;
    rhs_diff.points[i] = rhs.points[i] - center_for_wall_comparer;
  }
  
  double lhs_angles[2], rhs_angles[2];
  for (int i = 0; i < 2; ++i) {
    lhs_angles[i] = atan2(lhs_diff.points[i][1], lhs_diff.points[i][0]);
    rhs_angles[i] = atan2(rhs_diff.points[i][1], rhs_diff.points[i][0]);
  }
}

void SortWalls(const Eigen::Vector3d& center, Walls* walls) {
  

}

void SortRooms(const Eigen::Vector3d& center, vector<Walls>* rooms) {

}
  
  
} // namespace

PolygonRenderer::PolygonRenderer() {
}

PolygonRenderer::~PolygonRenderer() {
}

void PolygonRenderer::RenderWireframe(const int room, const double alpha) {
  if (room < 0 ||
      static_cast<int>(line_floorplan.line_rooms.size()) <= room) {
    cerr << "Index out of bounds: " << room << endl;
    exit (1);
  }
  const LineRoom& line_room = line_floorplan.line_rooms[room];

  // Floor.
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glBegin(GL_LINES);
  glColor4f(0.0f, 1.0f, 1.0f, alpha);
  for (int c = 0; c < static_cast<int>(line_room.walls.size()); ++c) {
    const int nextc = (c + 1) % (static_cast<int>(line_room.walls.size()));

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
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}

void PolygonRenderer::RenderWireframeAll(const double alpha) {
  for (int r = 0; r < static_cast<int>(line_floorplan.line_rooms.size()); ++r) {
    RenderWireframe(r, alpha);
  }
}

void PolygonRenderer::Init(const string data_directory) {
  file_io::FileIO file_io(data_directory);
  
  ifstream ifstr;
  ifstr.open(file_io.GetLineFloorplan().c_str());
  ifstr >> line_floorplan;
  ifstr.close();

  {
    ifstream ifstr;
    ifstr.open(file_io.GetRotationMat().c_str());
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        ifstr >> rotation(y, x);
      }
    }
    ifstr.close();
  }
}

void PolygonRenderer::RenderWallAll() {
  

  

}

void PolygonRenderer::RenderWall(const int room) {

}
