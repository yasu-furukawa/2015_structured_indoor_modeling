#include <fstream>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

#include "../base/file_io.h"
#include "polygon_renderer.h"
#include "tree_organizer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

struct Wall {
  Eigen::Vector2d points[2];
  double floor_height;
  double ceiling_height;
  Eigen::Vector3d color;
  Eigen::Vector3i colori;
};

typedef vector<Wall> Walls;

  /*
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
  */

  /*
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
  */

// -1: lhs is close
//  1: rhs is close
//  0: cannot compare
  /*
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

  cerr << "Rare case. a bug..." << optical_center[0] << ' ' << optical_center[1] << endl;
  exit (1);
  return 0;
}
  */

void SortTriangles(const Eigen::Vector3d& center, Triangles* triangles) {
  // This is not exact, but should be fine.
  Triangles new_triangles;
  vector<pair<double, int> > weight_triangle_id(triangles->size());
  for (int t = 0; t < (int)triangles->size(); ++t) {
    const Vector3d triangle_center = (triangles->at(t).vertices[0] +
                                      triangles->at(t).vertices[1] +
                                      triangles->at(t).vertices[2]) / 3.0;
    weight_triangle_id[t] = make_pair((center - triangle_center).norm(), t);
  }
  sort(weight_triangle_id.rbegin(), weight_triangle_id.rend());

  new_triangles.resize(triangles->size());
  for (int t = 0; t < (int)triangles->size(); ++t)
    new_triangles[t] = triangles->at(weight_triangle_id[t].second);

  triangles->swap(new_triangles);
}

  /*
void SortWalls(const Eigen::Vector2d& center, Walls* walls) {
  for (int i = 0; i < (int)walls->size(); ++i) {
    for (int j = i + 1; j < (int)walls->size(); ++j) {
      if (FrontBackTest(center, walls->at(i), walls->at(j)) == -1)
        swap(walls->at(i), walls->at(j));
    }
  }
}
  */
  //void SortRooms(const Eigen::Vector2d& center, vector<Walls>* rooms) {
  //}

Vector3d GenerateRoomColor(const int room) {
  switch (room) {
  case 0:
    return Vector3d(255, 179, 0) / 255.0;
  case 1:
    return Vector3d(0.0, 1.0, 1.0);
  case 2:
    return Vector3d(0.0, 0.0, 1.0);
  case 3:
    return Vector3d(255, 104, 0) / 255.0;
  case 4:
    return Vector3d(0.7, 1.0, 0.0);
  case 5:
    return Vector3d(0.0, 1.0, 0.0);
  case 6:
    return Vector3d(128, 62, 117) / 255.0;
  case 7:
    return Vector3d(193, 0, 32) / 255.0;
  case 8:
    return Vector3d(206, 162, 98) / 255.0;
  case 9:
    return Vector3d(166, 189, 215) / 255.0;
  case 10:
    return Vector3d(129, 112, 102) / 255.0;
  default:
    return Vector3d(1.0, 1.0, 1.0);
  }
}
  
} // namespace


PolygonRenderer::PolygonRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
}

PolygonRenderer::~PolygonRenderer() {
  for (int t = 0; t < (int)texture_ids.size(); ++t)
    widget->deleteTexture(texture_ids[t]);
}

void PolygonRenderer::RenderWireframe(const int /*room*/, const double /*alpha*/) {
  /*
  if (room < 0 ||
      static_cast<int>(floorplan.line_rooms.size()) <= room) {
    cerr << "Index out of bounds: " << room << endl;
    exit (1);
  }
  const LineRoom& line_room = floorplan.line_rooms[room];

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
    
    floor0 = floorplan.floorplan_to_global * floor0;
    floor1 = floorplan.floorplan_to_global * floor1;
    ceiling0 = floorplan.floorplan_to_global * ceiling0;
    ceiling1 = floorplan.floorplan_to_global * ceiling1;

    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);

    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
    glVertex3f(ceiling1[0], ceiling1[1], ceiling1[2]);

    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
  }
  glEnd();
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  */
}

void PolygonRenderer::RenderWireframeAll(const double alpha) {
  for (int r = 0; r < floorplan.GetNumRooms(); ++r) {
    RenderWireframe(r, alpha);
  }
}

void PolygonRenderer::Init(const string data_directory, QGLWidget* widget_tmp) {
  widget = widget_tmp;
  
  FileIO file_io(data_directory);

  int num_texture_images;
  for (num_texture_images = 0; ; ++num_texture_images) {
    ifstream ifstr;
    ifstr.open(file_io.GetTextureImage(num_texture_images).c_str());
    if (!ifstr.is_open())
      break;
  }
  texture_images.resize(num_texture_images);
  for (int t = 0; t < num_texture_images; ++t) {
    texture_images[t].load(file_io.GetTextureImage(t).c_str());
  }
}

void PolygonRenderer::InitGL() {
  initializeGLFunctions();

  texture_ids.resize(texture_images.size());
  
  glEnable(GL_TEXTURE_2D);
  for (int t = 0; t < (int)texture_images.size(); ++t) {
    texture_ids[t] = widget->bindTexture(texture_images[t]);
  }
}

void PolygonRenderer::RenderTextureMappedRooms(const double top_alpha, const double bottom_alpha) const {
  // For each texture.
  for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
    glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBegin(GL_TRIANGLES);
    
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
        const Vector3d v00 = floorplan.GetFloorVertexGlobal(room, wall);
        const Vector3d v10 = floorplan.GetFloorVertexGlobal(room, next_wall);
        const Vector3d v01 = floorplan.GetCeilingVertexGlobal(room, wall);
        const Vector3d v11 = floorplan.GetCeilingVertexGlobal(room, next_wall);
        const Vector3d x_diff = v10 - v00;
        const Vector3d y_diff = v01 - v00;
        
        const WallTriangulation wall_triangulation =
          floorplan.GetWallTriangulation(room, wall);

        for (const auto& triangle : wall_triangulation.triangles) {
          if (triangle.image_index != texture)
            continue;

          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            const int index = triangle.indices[i];
            const Vector2d vertex_in_uv = wall_triangulation.vertices_in_uv[index];

            Vector3d position;
            if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 0.0)
              position = v00;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 0.0)
              position = v10;
            else if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 1.0)
              position = v01;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 1.0)
              position = v11;
            else
              position = v00 + x_diff * vertex_in_uv[0] + y_diff * vertex_in_uv[1];

            const double alpha = vertex_in_uv[1] * (top_alpha - bottom_alpha) + bottom_alpha;
            glColor4f(alpha, alpha, alpha, 1.0);
            
            glVertex3d(position[0], position[1], position[2]);
          }
        }
      }

      // Floor.
      const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
      for (const auto& triangle : floor_triangulation.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
          // Make floor darker ?.
          // glColor4f(alpha / 2.0, alpha / 2.0, alpha / 2.0, 1.0);
          const int index = triangle.indices[i];
          const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
          glVertex3d(position[0], position[1], position[2]);
        }
      }
    }
    glEnd();
  }

  /*
  {
    glDisable(GL_TEXTURE_2D);
    // Floor.
    glBegin(GL_TRIANGLES);
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
      for (const auto& triangle : floor_triangulation.triangles) {
        for (int i = 0; i < 3; ++i) {
          // glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
          glColor4f(0.5, 0.5, 0.5, 1.0);
          const int index = triangle.indices[i];
          const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
          glVertex3d(position[0], position[1], position[2]);
        }
      }
    }
    glEnd();
    glEnable(GL_TEXTURE_2D);
  }
  */
}

void PolygonRenderer::RenderDoors(const double alpha) const {
   glBegin(GL_QUADS);
   glColor4f(alpha, alpha, alpha, 1.0);
   for (int door = 0; door < floorplan.GetNumDoors(); ++door) {
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 0)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 1)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 4)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 5)[0]);

     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 1)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 2)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 7)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 4)[0]);

     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 2)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 3)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 6)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 7)[0]);

     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 3)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 0)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 5)[0]);
     glVertex3dv(&floorplan.GetDoorVertexGlobal(door, 6)[0]);
   }
   glEnd();
}    


void PolygonRenderer::RenderTextureMappedRooms(const double top_alpha,
                                               const double bottom_alpha,
                                               const TreeOrganizer& tree_organizer,
                                               const double air_to_tree_progress,
                                               const double animation,
                                               const Eigen::Vector3d& max_vertical_shift,
                                               const double max_shrink_ratio) const {
  // For each texture.
  for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
    glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBegin(GL_TRIANGLES);
    
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
        const Vector3d v00 = floorplan.GetFloorVertexGlobal(room, wall);
        const Vector3d v10 = floorplan.GetFloorVertexGlobal(room, next_wall);
        const Vector3d v01 = floorplan.GetCeilingVertexGlobal(room, wall);
        const Vector3d v11 = floorplan.GetCeilingVertexGlobal(room, next_wall);
        const Vector3d x_diff = v10 - v00;
        const Vector3d y_diff = v01 - v00;
        
        const WallTriangulation wall_triangulation =
          floorplan.GetWallTriangulation(room, wall);

        for (const auto& triangle : wall_triangulation.triangles) {
          if (triangle.image_index != texture)
            continue;

          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            const int index = triangle.indices[i];
            const Vector2d vertex_in_uv = wall_triangulation.vertices_in_uv[index];

            Vector3d position;
            if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 0.0)
              position = v00;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 0.0)
              position = v10;
            else if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 1.0)
              position = v01;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 1.0)
              position = v11;
            else
              position = v00 + x_diff * vertex_in_uv[0] + y_diff * vertex_in_uv[1];

            const double alpha = vertex_in_uv[1] * (top_alpha - bottom_alpha) + bottom_alpha;
            glColor4f(alpha, alpha, alpha, 1.0);

            // Move wall vertex closer to the centers.
            position = tree_organizer.TransformRoom(position,
                                                    room,
                                                    air_to_tree_progress,
                                                    animation,
                                                    max_vertical_shift);
            glVertex3d(position[0], position[1], position[2]);
          }
        }
      }

      // Floor.
      const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
      for (const auto& triangle : floor_triangulation.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
          // Make floor darker ?.
          // glColor4f(alpha / 2.0, alpha / 2.0, alpha / 2.0, 1.0);
          const int index = triangle.indices[i];
          Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
          position = tree_organizer.TransformRoom(position,
                                                  room,
                                                  air_to_tree_progress,
                                                  animation,
                                                  max_vertical_shift);
          
          glVertex3d(position[0], position[1], position[2]);
        }
      }
    }
    glEnd();
  }

  /*
  const double max_shrink_ratio2 = tree_organizer.GetFloorplanDeformation().shrink_ratio;

  const double shrink_ratio = air_to_tree_progress * max_shrink_ratio2 + (1.0 - air_to_tree_progress);
  const Vector3d vertical_shift(0, 0, air_to_tree_progress * max_vertical_shift);
  Matrix3d rotation;
  rotation(0, 0) = cos(animation * 2 * M_PI);
  rotation(0, 1) = -sin(animation * 2 * M_PI);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(animation * 2 * M_PI);
  rotation(1, 1) = cos(animation * 2 * M_PI);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;

  const std::vector<Eigen::Vector3d>& displacements =
    tree_organizer.GetFloorplanDeformation().displacements;

  // For each texture.
  for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
    glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBegin(GL_TRIANGLES);
    
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      const BoundingBox& bounding_box = tree_organizer.GetFloorplanDeformation().room_bounding_boxes[room];
      const Vector3d room_center = (bounding_box.min_xyz + bounding_box.max_xyz) / 2.0;
        
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
        const Vector3d v00 = floorplan.GetFloorVertexGlobal(room, wall);
        const Vector3d v10 = floorplan.GetFloorVertexGlobal(room, next_wall);
        const Vector3d v01 = floorplan.GetCeilingVertexGlobal(room, wall);
        const Vector3d v11 = floorplan.GetCeilingVertexGlobal(room, next_wall);
        const Vector3d x_diff = v10 - v00;
        const Vector3d y_diff = v01 - v00;
        
        const WallTriangulation wall_triangulation =
          floorplan.GetWallTriangulation(room, wall);

        for (const auto& triangle : wall_triangulation.triangles) {
          if (triangle.image_index != texture)
            continue;

          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            const int index = triangle.indices[i];
            const Vector2d vertex_in_uv = wall_triangulation.vertices_in_uv[index];

            Vector3d position;
            if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 0.0)
              position = v00;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 0.0)
              position = v10;
            else if (vertex_in_uv[0] == 0.0 && vertex_in_uv[1] == 1.0)
              position = v01;
            else if (vertex_in_uv[0] == 1.0 && vertex_in_uv[1] == 1.0)
              position = v11;
            else
              position = v00 + x_diff * vertex_in_uv[0] + y_diff * vertex_in_uv[1];

            const double alpha = vertex_in_uv[1] * (top_alpha - bottom_alpha) + bottom_alpha;
            glColor4f(alpha, alpha, alpha, 1.0);

            // Move wall vertex closer to the centers.
            position = room_center + rotation * (position - room_center) * shrink_ratio;
            position += displacements[room] + vertical_shift;
            glVertex3d(position[0], position[1], position[2]);
          }
        }
      }

      // Floor.
      const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
      for (const auto& triangle : floor_triangulation.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
          // Make floor darker ?.
          // glColor4f(alpha / 2.0, alpha / 2.0, alpha / 2.0, 1.0);
          const int index = triangle.indices[i];
          Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
          position = room_center + rotation * (position - room_center) * shrink_ratio;
          position += displacements[room] + vertical_shift;
          glVertex3d(position[0], position[1], position[2]);
        }
      }
    }
    glEnd();
  }
  */
}

void PolygonRenderer::RenderDoors(const double alpha,
                                  const TreeOrganizer& tree_organizer,
                                  const double air_to_tree_progress,
                                  const double animation,
                                  const Eigen::Vector3d& max_vertical_shift,
                                  const double max_shrink_ratio) const {
  //???
}
  
void PolygonRenderer::SetTargetCeilingHeights(const Eigen::Vector3d& center,
                                              const bool depth_order_height_adjustment,
                                              const int room_not_rendered,
                                              std::vector<double>* target_ceiling_heights) {
  target_ceiling_heights->resize(floorplan.GetNumRooms());
  if (!depth_order_height_adjustment) {    
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      target_ceiling_heights->at(room) = floorplan.GetFloorHeight(room);
      return;
    }
  }
  
  const Vector3d local_center = floorplan.GetFloorplanToGlobal().transpose() * center;

  vector<double> distances(floorplan.GetNumRooms(), 0.0);
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    distances[room] = (Vector2d(local_center[0], local_center[1]) -
                       floorplan.GetRoomCenterLocal(room)).norm();
  }
  vector<int> room_orders(floorplan.GetNumRooms(), -1);
  {
    vector<pair<double, int> > distances_room;
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      if (room == room_not_rendered)
        continue;
      distances_room.push_back(pair<double, int>(distances[room], room));
    }
    sort(distances_room.begin(), distances_room.end());
    for (int i = 0; i < (int)distances_room.size(); ++i) {
      room_orders[distances_room[i].second] = i;
    }
  }
  double average_length = 0.0;
  {
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      average_length += floorplan.GetCeilingHeight(room) -
        floorplan.GetFloorHeight(room);
    }
    average_length /= floorplan.GetNumRooms();
  }
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const double target_length =
      average_length * (room_orders[room] + 1) / (floorplan.GetNumRooms() - 1);
    target_ceiling_heights->at(room) = floorplan.GetFloorHeight(room) + target_length;
  }
}

void PolygonRenderer::AddTrianglesFromWalls(const double height_adjustment,
                                            const int room_not_rendered,
                                            const int room_highlighted,
                                            const std::vector<double>& target_ceiling_heights,
                                            Triangles* triangles) {
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  // Add triangles from walls.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    if (room == room_not_rendered)
      continue;
    
    Vector3d color = GenerateRoomColor(room);
    if (room_highlighted == room)
      color = Vector3d(1, 1, 1);
    const Vector3i colori(0, 0, room + 1);
    
    // const LineRoom& line_room = floorplan.line_rooms[room];
    const int num_walls = floorplan.GetNumWalls(room);
    for (int w = 0; w < num_walls; ++w) {
      const int next_w = (w + 1) % num_walls;
      const Vector2d v0 = floorplan.GetRoomVertexLocal(room, w);
      const Vector2d v1 = floorplan.GetRoomVertexLocal(room, next_w);
      
      const double ceiling_height = floorplan.GetCeilingHeight(room) + (target_ceiling_heights[room] - floorplan.GetCeilingHeight(room)) * height_adjustment;
      
      Triangle2 triangle0, triangle1;
      triangle0.vertices[0] = floorplan_to_global * Vector3d(v0[0], v0[1], floorplan.GetFloorHeight(room));
      triangle0.vertices[1] = floorplan_to_global * Vector3d(v1[0], v1[1], floorplan.GetFloorHeight(room));
      triangle0.vertices[2] = floorplan_to_global * Vector3d(v1[0], v1[1], ceiling_height);
      
      triangle1.vertices[0] = floorplan_to_global * Vector3d(v0[0], v0[1], floorplan.GetFloorHeight(room));
      triangle1.vertices[1] = floorplan_to_global * Vector3d(v1[0], v1[1], ceiling_height);
      triangle1.vertices[2] = floorplan_to_global * Vector3d(v0[0], v0[1], ceiling_height);
      
      triangle0.colors[0] = color / 3;
      triangle0.colors[1] = color / 3;
      triangle0.colors[2] = color;
      triangle0.colori = colori;
      triangle1.colors[0] = color / 3;
      triangle1.colors[1] = color;
      triangle1.colors[2] = color;
      triangle1.colori = colori;
      triangles->push_back(triangle0);
      triangles->push_back(triangle1);
    }
  }
}

void PolygonRenderer::AddTrianglesFromCeiling(const double height_adjustment,
                                              const int room_not_rendered,
                                              const int room_highlighted,
                                              const std::vector<double>& target_ceiling_heights,
                                              Triangles* triangles) {
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    if (room == room_not_rendered)
      continue;
    
    const double ceiling_height =
      floorplan.GetCeilingHeight(room) +
      (target_ceiling_heights[room] - floorplan.GetCeilingHeight(room)) *
      height_adjustment;
    
    const FloorCeilingTriangulation& ceiling_triangulation = floorplan.GetCeilingTriangulation(room);
    for (const auto& triangle : ceiling_triangulation.triangles) {
      Triangle2 triangle2;
      for (int i = 0; i < 3; ++i) {
        const Vector2d xy_local = floorplan.GetRoomVertexLocal(room, triangle.indices[i]);
        triangle2.vertices[2 - i] =
          floorplan_to_global * Vector3d(xy_local[0], xy_local[1], ceiling_height);
      }
      if (room_highlighted == room) {
        for (int i = 0; i < 3; ++i)
          triangle2.colors[i] = Vector3d(1, 1, 1);
      } else {
        for (int i = 0; i < 3; ++i)
          triangle2.colors[i] = GenerateRoomColor(room);
      }
      triangle2.colori = Vector3i(0, 0, room + 1);
      
      triangles->push_back(triangle2);
    }
  }
}

void PolygonRenderer::RenderWallAll(const Eigen::Vector3d& center,
                                    const double alpha,
                                    const double height_adjustment,
                                    const bool depth_order_height_adjustment,
                                    const int room_not_rendered,
                                    const int room_highlighted,
                                    const bool render_room_id) {
  vector<double> target_ceiling_heights;
  SetTargetCeilingHeights(center,
                          depth_order_height_adjustment,
                          room_not_rendered,
                          &target_ceiling_heights);

  // const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  Triangles triangles;
  AddTrianglesFromWalls(height_adjustment,
                        room_not_rendered,
                        room_highlighted,
                        target_ceiling_heights,
                        &triangles);
  
  // Add triangles from ceiling.
  AddTrianglesFromCeiling(height_adjustment,
                          room_not_rendered,
                          room_highlighted,
                          target_ceiling_heights,
                          &triangles);  

  SortTriangles(center, &triangles);

  glBegin(GL_TRIANGLES);
  for (const auto& triangle : triangles) {
    for (int i = 0; i < 3; ++i) {
      if (render_room_id)
        glColor4ub(triangle.colori[0], triangle.colori[1], triangle.colori[2], 255);
      else
        glColor4f(triangle.colors[i][0], triangle.colors[i][1], triangle.colors[i][2], alpha);
      
      glVertex3d(triangle.vertices[i][0], triangle.vertices[i][1], triangle.vertices[i][2]);
    }
  }
  glEnd();


  /*
  const Vector3d local_center = floorplan.GetFloorplanToGlobal().transpose() * center;

  vector<double> distances(room_centers_local.size(), 0.0);
  for (int room = 0; room < (int)room_centers_local.size(); ++room) {
    distances[room] = (Vector2d(local_center[0], local_center[1]) -
                       room_centers_local[room]).norm();
  }
  vector<int> room_orders(room_centers_local.size(), -1);
  {
    vector<pair<double, int> > distances_room;
    for (int room = 0; room < (int)room_centers_local.size(); ++room) {
      if (room == room_not_rendered)
        continue;
      distances_room.push_back(pair<double, int>(distances[room], room));
    }
    sort(distances_room.begin(), distances_room.end());
    for (int i = 0; i < (int)distances_room.size(); ++i) {
      room_orders[distances_room[i].second] = i;
    }
  }

  double average_length = 0.0;
  {
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      average_length += floorplan.GetCeilingHeight(room) -
        floorplan.GetFloorHeight(room);
    }
    average_length /= floorplan.GetNumRooms();
  }
  
  Walls walls;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    if (room == room_not_rendered)
      continue;

    Vector3d color = GenerateRoomColor(room);
    if (room_highlighted == room)
      color = Vector3d(1, 1, 1);
    const Vector3i colori(0, 0, room + 1);
    
    // const LineRoom& line_room = floorplan.line_rooms[room];
    const int num_walls = floorplan.GetNumWalls(room);
    for (int w = 0; w < num_walls; ++w) {
      const int next_w = (w + 1) % num_walls;
      Wall wall;
      wall.points[0] = floorplan.GetRoomVertexLocal(room, w);
      wall.points[1] = floorplan.GetRoomVertexLocal(room, next_w);
      wall.floor_height   = floorplan.GetFloorHeight(room);
      //????
      //      wall.floor_height   = line_room.ceiling_height + (line_room.floor_height - line_room.ceiling_height) * 0.1;

      double target_length;
      if (depth_order_height_adjustment) {
        target_length =
          average_length * (room_orders[room] + 1) / ((int)room_centers_local.size() - 1);
      }
      else {
        target_length = average_length * 0.2;
      }
      const double target_ceiling_height = floorplan.GetFloorHeight(room) + target_length;

      wall.ceiling_height = floorplan.GetCeilingHeight(room) + (target_ceiling_height - floorplan.GetCeilingHeight(room)) * height_adjustment;
      
      wall.color = color;
      wall.colori = colori;
      walls.push_back(wall);
    }
  }
  
  SortWalls(Vector2d(local_center[0], local_center[1]), &walls);

  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
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
    
    floor0   = floorplan_to_global * floor0;
    floor1   = floorplan_to_global * floor1;
    ceiling0 = floorplan_to_global * ceiling0;
    ceiling1 = floorplan_to_global * ceiling1;

    if (render_room_id)
      glColor4ub(wall.colori[0], wall.colori[1], wall.colori[2], 255);
    else
      glColor4f(wall.color[0], wall.color[1], wall.color[2], alpha);
    
    glVertex3f(floor0[0], floor0[1], floor0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);
    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);

    glVertex3f(ceiling0[0], ceiling0[1], ceiling0[2]);
    glVertex3f(floor1[0], floor1[1], floor1[2]);
    glVertex3f(ceiling1[0], ceiling1[1], ceiling1[2]);
  }
  glEnd();
  */
}

}  // namespace structured_indoor_modeling
