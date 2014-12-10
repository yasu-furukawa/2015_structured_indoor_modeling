#include <fstream>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

#include "../calibration/file_io.h"
#include "polygon_renderer.h"

using namespace Eigen;
using namespace std;

namespace {

struct Wall {
  Eigen::Vector2d points[2];
  double floor_height;
  double ceiling_height;
  Eigen::Vector3d color;
};

typedef vector<Wall> Walls;

void wall_to_angles(const Eigen::Vector2d& optical_center,
                    const Wall& wall,
                    double angles[2]) {
  for (int i = 0; i < 2; ++i) {
    Eigen::Vector2d diff = wall.points[i] - optical_center;
    angles[i] = atan2(diff[1], diff[0]);
  }

  const double min_angle = min(angles[0], angles[1]);
  const double max_angle = max(angles[0], angles[1]);

  if (max_angle - min_angle > M_PI) {
    if (angles[0] < angles[1])
      angles[0] += 2.0 * M_PI;
    else
      angles[1] += 2.0 * M_PI;
  }
}

double ComputeDistance(const Eigen::Vector2d& center,
                       const Eigen::Vector2d& target,
                       const Wall& wall) {
  // center -> target must intersect with wall.
  Vector2d diff0 = target - center;
  Vector2d diff1 = wall.points[1] - wall.points[0];

  // center + s (diff0) = wall.points[0] + t (diff1)
  // (diff0 -diff1) (s)  = wall.points[0] - center
  // (            ) (t)
  
  Matrix2d A;
  A << diff0[0], -diff1[0], diff0[1], -diff1[1];
  Vector2d b = wall.points[0] - center;

  double t;
  if (A.determinant() == 0) {
    t = 0;
  } else {
    Vector2d x = A.ldlt().solve(b);
    t = x[1];
  }

  return (wall.points[0] + t * diff1 - center).norm();
}

// -1: lhs is close
//  1: rhs is close
//  0: cannot compare
int FrontBackTest(const Eigen::Vector2d& optical_center, const Wall& lhs, const Wall& rhs) {
  double lhs_angles[2], rhs_angles[2];
  wall_to_angles(optical_center, lhs, lhs_angles);
  wall_to_angles(optical_center, rhs, rhs_angles);

  const double min_lhs_angle = min(lhs_angles[0], lhs_angles[1]);
  const double max_lhs_angle = max(lhs_angles[0], lhs_angles[1]);
  const double min_rhs_angle = min(rhs_angles[0], rhs_angles[1]);
  const double max_rhs_angle = max(rhs_angles[0], rhs_angles[1]);
  // Checks if there is an overlap.
  if (max_lhs_angle < min_rhs_angle || max_rhs_angle < min_lhs_angle)
    return 0;

  // One of the 4 corners must be in the common range.
  for (int i = 0; i < 2; ++i) {
    // If lhs_points[i] is in the common range.
    if (min_rhs_angle <= lhs_angles[i] && lhs_angles[i] <= max_rhs_angle) {
      const double ref_distance = (lhs.points[i] - optical_center).norm();

      const double distance = ComputeDistance(optical_center, lhs.points[i], rhs);
      if (ref_distance < distance)
        return -1;
      else
        return 1;
    }
  }

  for (int i = 0; i < 2; ++i) {
    // If rhs_points[i] is in the common range.
    if (min_lhs_angle <= rhs_angles[i] && rhs_angles[i] <= max_lhs_angle) {
      const double ref_distance = (rhs.points[i] - optical_center).norm();

      const double distance = ComputeDistance(optical_center, rhs.points[i], lhs);
      if (ref_distance < distance)
        return 1;
      else
        return -1;
    }
  }

  cerr << "Rare case. a bug..." << endl;
  return 0;
}

void SortWalls(const Eigen::Vector2d& center, Walls* walls) {
  for (int i = 0; i < (int)walls->size(); ++i) {
    for (int j = i + 1; j < (int)walls->size(); ++j) {
      if (FrontBackTest(center, walls->at(i), walls->at(j)) == -1)
        swap(walls->at(i), walls->at(j));
    }
  }
}

  //void SortRooms(const Eigen::Vector2d& center, vector<Walls>* rooms) {
  //}

Vector3d GenerateRoomColor(const int room) {
  switch (room) {
  case 0:
    return Vector3d(255, 179, 0) / 255.0;
  case 1:
    return Vector3d(129, 112, 102) / 255.0;
  case 2:
    return Vector3d(0.0, 0.0, 1.0);
  case 3:
    return Vector3d(166, 189, 215) / 255.0;
  case 4:
    return Vector3d(193, 0, 32) / 255.0;
  case 5:
    return Vector3d(0.0, 1.0, 0.0);
  case 6:
    return Vector3d(128, 62, 117) / 255.0;
  case 7:
    return Vector3d(0.7, 1.0, 0.0);
  case 8:
    return Vector3d(206, 162, 98) / 255.0;
  case 9:
    return Vector3d(255, 104, 0) / 255.0;
  case 10:
    return Vector3d(0.0, 1.0, 1.0);
  default:
    return Vector3d(1.0, 1.0, 1.0);
  }
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
    
    floor0 = floorplan_to_global * floor0;
    floor1 = floorplan_to_global * floor1;
    ceiling0 = floorplan_to_global * ceiling0;
    ceiling1 = floorplan_to_global * ceiling1;

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
        ifstr >> floorplan_to_global(y, x);
      }
    }
    ifstr.close();
  }

  //----------------------------------------------------------------------
  room_centers.resize(line_floorplan.line_rooms.size());
  for (int room = 0; room < (int)room_centers.size(); ++room) {
    const LineRoom& line_room = line_floorplan.line_rooms[room];
    
    room_centers[room] = Vector2d(0, 0);
    for (const auto& wall : line_room.walls) {
      room_centers[room] += wall;
    }
    if (line_room.walls.empty()) {
      cerr << "Impossible." << endl;
      exit (1);
    }
    room_centers[room] /= line_room.walls.size();
  }  
}

void PolygonRenderer::RenderWallAll(const Eigen::Vector3d& center,
                                    const double alpha,
                                    const double height_adjustment,
                                    const int center_room) {
  const Vector3d local_center = floorplan_to_global.transpose() * center;

  vector<double> distances(room_centers.size(), 0.0);
  double min_distance = numeric_limits<double>::max();
  double max_distance = -1.0;
  for (int room = 0; room < (int)room_centers.size(); ++room) {
    distances[room] = (Vector2d(local_center[0], local_center[1]) -
                       room_centers[room]).norm();
    if (room != center_room) {
      min_distance = min(min_distance, distances[room]);
      max_distance = max(max_distance, distances[room]);
    }
  }

  vector<double> height_adjustment_rates(room_centers.size());
  for (int room = 0; room < (int)room_centers.size(); ++room) {
    height_adjustment_rates[room] =
      max(0.0, min(0.8, (distances[room] - max_distance) / (min_distance - max_distance)));
  }
  
  Walls walls;
  for (int room = 0; room < (int)line_floorplan.line_rooms.size(); ++room) {
    if (room == center_room)
      continue;

    const Vector3d color = GenerateRoomColor(room);
    
    const LineRoom& line_room = line_floorplan.line_rooms[room];
    for (int w = 0; w < (int)line_room.walls.size(); ++w) {
      const int next_w = (w + 1) % line_room.walls.size();
      Wall wall;
      wall.points[0] = line_room.walls[w];
      wall.points[1] = line_room.walls[next_w];
      wall.ceiling_height   = line_room.ceiling_height;
      //????
      //      wall.floor_height   = line_room.ceiling_height + (line_room.floor_height - line_room.ceiling_height) * 0.1;
      wall.floor_height = line_room.floor_height +
        (line_room.ceiling_height - line_room.floor_height) *
        height_adjustment * height_adjustment_rates[room];
      
      wall.color = color;
      walls.push_back(wall);
    }
  }

  
  SortWalls(Vector2d(local_center[0], local_center[1]), &walls);

  glBegin(GL_TRIANGLES);
  for (const auto& wall : walls) {
    Eigen::Vector3d floor0(wall.points[0][0],
                           wall.points[0][1],
                           wall.floor_height);
    Eigen::Vector3d floor1(wall.points[1][0],
                           wall.points[1][1],
                           wall.floor_height);

    Eigen::Vector3d ceiling0(wall.points[0][0],
                           wall.points[0][1],
                           wall.ceiling_height);
    Eigen::Vector3d ceiling1(wall.points[1][0],
                           wall.points[1][1],
                           wall.ceiling_height);
    
    floor0 = floorplan_to_global * floor0;
    floor1 = floorplan_to_global * floor1;
    ceiling0 = floorplan_to_global * ceiling0;
    ceiling1 = floorplan_to_global * ceiling1;

    glColor4f(wall.color[0], wall.color[1], wall.color[2], alpha);
    
    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);
    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);

    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);
    glVertex3f(ceiling1[0], ceiling1[1], ceiling1[2]);
  }
  glEnd();
}

void PolygonRenderer::RenderWall(const int room) {

}
