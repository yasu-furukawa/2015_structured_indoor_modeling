#include <fstream>
#include <iostream>
#include <numeric>
#include <map>
#include <set>
#include <Eigen/Dense>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "indoor_polygon_renderer.h"
#include "navigation.h"
#include "view_parameters.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

IndoorPolygonRenderer::IndoorPolygonRenderer(const Floorplan& floorplan,
                                             const IndoorPolygon& indoor_polygon,
                                             const Navigation& navigation)
  : floorplan(floorplan), indoor_polygon(indoor_polygon), navigation(navigation) {
  render_mode = kBackWallFaceTransparent;
}

IndoorPolygonRenderer::~IndoorPolygonRenderer() {
  for (int t = 0; t < (int)texture_ids.size(); ++t)
    widget->deleteTexture(texture_ids[t]);
}


void IndoorPolygonRenderer::Init(const string& data_directory,
                                 const std::string& suffix,
                                 QGLWidget* widget_tmp) {
  widget = widget_tmp;
  
  FileIO file_io(data_directory);

  int num_texture_images;
  for (num_texture_images = 0; ; ++num_texture_images) {
    ifstream ifstr;
    ifstr.open(file_io.GetTextureImageIndoorPolygon(num_texture_images, suffix).c_str());
    if (!ifstr.is_open())
      break;
  }
  texture_images.resize(num_texture_images);
  for (int t = 0; t < num_texture_images; ++t) {
    texture_images[t].load(file_io.GetTextureImageIndoorPolygon(t, suffix).c_str());
  }

  //----------------------------------------------------------------------
  {
    bool first = true;
    for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
      const Segment& segment = indoor_polygon.GetSegment(s);
      for (const auto& vertex : segment.vertices) {
        if (first) {
          bottom_z = top_z = vertex[2];
          first = false;
        } else {
          bottom_z = min(bottom_z, vertex[2]);
          top_z    = max(top_z, vertex[2]);
        }
      }
    }
  }

  //----------------------------------------------------------------------
  {
    map<int, Segment> floor_segments;
    map<int, Segment> ceiling_segments;

    for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
      const Segment& segment = indoor_polygon.GetSegment(s);
      if (segment.type == Segment::FLOOR) {
        floor_segments[segment.floor_info] = segment;
      } else if (segment.type == Segment::CEILING) {
        ceiling_segments[segment.ceiling_info] = segment;
      }
    }

    for (const auto& item : floor_segments) {
      const int room = item.first;
      const Segment& floor_segment = item.second;
      const Segment& ceiling_segment = ceiling_segments[room];
      const double ceiling_height = indoor_polygon.GlobalToManhattan(ceiling_segment.vertices[0])[2];

      std::set<pair<int, int> > floor_edges;
      for (const auto& triangle : floor_segment.triangles) {
        for (int i = 0; i < 3; ++i) {
          const int v0 = triangle.indices[i];
          const int v1 = triangle.indices[(i + 1) % 3];
          const pair<int, int> target = make_pair(v1, v0);
          if (floor_edges.find(target) != floor_edges.end())
            floor_edges.erase(target);
          else
            floor_edges.insert(make_pair(v0, v1));
        }
      }

      for (const auto& edge : floor_edges) {
        const Vector3d floor_local_v0 = floor_segment.vertices[edge.first];
        const Vector3d floor_local_v1 = floor_segment.vertices[edge.second];
        const Vector3d ceiling_local_v0(floor_local_v0[0], floor_local_v0[1], ceiling_height);
        const Vector3d ceiling_local_v1(floor_local_v1[0], floor_local_v1[1], ceiling_height);

        vector<Vector3d> wire_frame;
        wire_frame.push_back(indoor_polygon.ManhattanToGlobal(floor_local_v0));
        wire_frame.push_back(indoor_polygon.ManhattanToGlobal(floor_local_v1));
        wire_frame.push_back(indoor_polygon.ManhattanToGlobal(ceiling_local_v1));
        wire_frame.push_back(indoor_polygon.ManhattanToGlobal(ceiling_local_v0));
        wire_frames[room].push_back(wire_frame);
      }
    }
  }
}
  
void IndoorPolygonRenderer::InitGL() {
  initializeGLFunctions();

  texture_ids.resize(texture_images.size());
  
  glEnable(GL_TEXTURE_2D);
  for (int t = 0; t < (int)texture_images.size(); ++t) {
    texture_ids[t] = widget->bindTexture(texture_images[t]);
  }
}

void IndoorPolygonRenderer::ToggleRenderMode() {
  vector<RenderMode> render_modes;
  render_modes.push_back(kFull);
  render_modes.push_back(kBackWallFaceCulling);
  render_modes.push_back(kBackWallFaceTransparent);

  for (int r = 0; r < (int)render_modes.size(); ++r) {
    if (render_modes[r] == render_mode) {
      render_mode = render_modes[(r + 1) % render_modes.size()];
      break;
    }
  }
}  

void IndoorPolygonRenderer::RenderTextureMappedRooms(const double top_intensity,
                                                     const double bottom_intensity) {
  vector<vector<bool> > render_for_room_wall;
  render_for_room_wall.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    render_for_room_wall[room].resize(floorplan.GetNumWalls(room), true);

    if (render_mode == kBackWallFaceCulling ||
        render_mode == kBackWallFaceTransparent) {
      const Vector3d& direction = navigation.GetDirection();
      for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
        const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
        const Vector3d diff0 = floorplan.GetFloorVertexGlobal(room, next_wall) -
          floorplan.GetFloorVertexGlobal(room, wall);
        const Vector3d diff1 = floorplan.GetCeilingVertexGlobal(room, wall) -
          floorplan.GetFloorVertexGlobal(room, wall);
        const Vector3d normal = diff0.cross(diff1);
        if (normal.dot(direction) > 0.0)
          render_for_room_wall[room][wall] = false;
      }
    }
  }

  if (render_mode == kFull || render_mode == kBackWallFaceCulling) {
    // For each texture.
    for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
      glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      
      glBegin(GL_TRIANGLES);
      for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
        const Segment& segment = indoor_polygon.GetSegment(s);
        if (segment.type == Segment::CEILING)
          continue;
        
        if (segment.type == Segment::WALL &&
            !render_for_room_wall[segment.wall_info[0]][segment.wall_info[1]])
          continue;
        
        for (const auto& triangle : segment.triangles) {
          if (triangle.image_index != texture)
            continue;
          
          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            
            const double z_position =
              max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
            const double intensity = z_position * (top_intensity - bottom_intensity) + bottom_intensity;
            glColor4f(intensity, intensity, intensity, 1.0);
            
            const Vector3d global =
              indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);
            glVertex3d(global[0], global[1], global[2]);
          }
        }
      }
      glEnd();
    }
  } else if (render_mode == kBackWallFaceTransparent) {
    // Sort rooms.
    // vector<int> sorted_rooms;

    for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
      glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      glBegin(GL_TRIANGLES);
      for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
        const Segment& segment = indoor_polygon.GetSegment(s);
        if (segment.type == Segment::CEILING)
          continue;

        if (segment.type == Segment::WALL &&
            !render_for_room_wall[segment.wall_info[0]][segment.wall_info[1]])
          continue;                

        for (const auto& triangle : segment.triangles) {
          if (triangle.image_index != texture)
            continue;
          
          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            
            const double z_position =
              max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
            const double intensity = z_position * (top_intensity - bottom_intensity) + bottom_intensity;
            glColor4f(intensity, intensity, intensity, 1.0);
            
            const Vector3d global =
              indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);
            glVertex3d(global[0], global[1], global[2]);
          }
        }
      }
      glEnd();
    }
    //----------------------------------------------------------------------
    glEnable(GL_BLEND);
    //glBlendColor(0.8, 0.8, 0.8, 1.0);
    // glBlendColor(0.5, 0.5, 0.5, 0.5);
    glBlendColor(0, 0, 0, 0.5);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);

    for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
      glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      
      glBegin(GL_TRIANGLES);
      for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
        const Segment& segment = indoor_polygon.GetSegment(s);

        if (!(segment.type == Segment::WALL &&
              !render_for_room_wall[segment.wall_info[0]][segment.wall_info[1]]))
          continue;
        
        for (const auto& triangle : segment.triangles) {
          if (triangle.image_index != texture)
            continue;
          
          for (int i = 0; i < 3; ++i) {
            glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);
            
            const double z_position =
              max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
            const double intensity = z_position * (top_intensity - bottom_intensity) + bottom_intensity;
            glColor4f(intensity, intensity, intensity, 1.0);
            
            const Vector3d global =
              indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);
            glVertex3d(global[0], global[1], global[2]);
          }
        }
      }
      glEnd();
    }
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
  }
}

void IndoorPolygonRenderer::RenderTextureMappedRooms(const double top_intensity,
                                                     const double bottom_intensity,
                                                     const ViewParameters& view_parameters,
                                                     const double air_to_tree_progress,
                                                     const double animation,
                                                     const Eigen::Vector3d& max_vertical_shift) const {
  // For each texture.
  for (int texture = 0; texture < (int)texture_ids.size(); ++texture) {
    glBindTexture(GL_TEXTURE_2D, texture_ids[texture]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBegin(GL_TRIANGLES);
    for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
      const Segment& segment = indoor_polygon.GetSegment(s);
      int room;
      int wall = -1;
      if (segment.type == Segment::CEILING) {
        continue;
      } else if (segment.type == Segment::DOOR) {
        continue;
      } else if (segment.type == Segment::FLOOR) {
        room = segment.floor_info;
      } else if (segment.type == Segment::WALL) {
        room = segment.wall_info[0];
        wall = segment.wall_info[1];
      } else {
        cerr << "Invalid" << endl;
        exit (1);
      }

      for (const auto& triangle : segment.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);

          const double z_position =
            max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
          const double intensity = z_position * (top_intensity - bottom_intensity) + bottom_intensity;
          glColor4f(intensity, intensity, intensity, 1.0);
          
          Vector3d global =
            indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);

          global = view_parameters.TransformRoom(global,
                                                room,
                                                air_to_tree_progress,
                                                animation,
                                                max_vertical_shift);
          
          glVertex3d(global[0], global[1], global[2]);
        }
      }
    }
    glEnd();
  }
}
  
}  // namespace structured_indoor_modeling
