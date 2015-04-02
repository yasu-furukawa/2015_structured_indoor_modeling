#include <fstream>
#include <iostream>
#include <numeric>
#include <map>
#include <set>
#include <Eigen/Dense>

#include "../base/file_io.h"
#include "../base/indoor_polygon.h"
#include "indoor_polygon_renderer.h"
#include "view_parameters.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

IndoorPolygonRenderer::IndoorPolygonRenderer(const IndoorPolygon& indoor_polygon)
  : indoor_polygon(indoor_polygon) {
}

IndoorPolygonRenderer::~IndoorPolygonRenderer() {
  for (int t = 0; t < (int)texture_ids.size(); ++t)
    widget->deleteTexture(texture_ids[t]);
}


void IndoorPolygonRenderer::Init(const string data_directory, QGLWidget* widget_tmp) {
  widget = widget_tmp;
  
  FileIO file_io(data_directory);

  int num_texture_images;
  for (num_texture_images = 0; ; ++num_texture_images) {
    ifstream ifstr;
    ifstr.open(file_io.GetTextureImageIndoorPolygon(num_texture_images).c_str());
    if (!ifstr.is_open())
      break;
  }
  texture_images.resize(num_texture_images);
  for (int t = 0; t < num_texture_images; ++t) {
    texture_images[t].load(file_io.GetTextureImageIndoorPolygon(t).c_str());
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

void IndoorPolygonRenderer::RenderTextureMappedRooms(const double top_alpha,
                                                     const double bottom_alpha) const {
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
      
      for (const auto& triangle : segment.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);

          const double z_position =
            max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
          const double alpha = z_position * (top_alpha - bottom_alpha) + bottom_alpha;
          glColor4f(alpha, alpha, alpha, 1.0);
          
          const Vector3d global =
            indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);
          glVertex3d(global[0], global[1], global[2]);
        }
      }
    }
    glEnd();
  }
}

void IndoorPolygonRenderer::RenderTextureMappedRooms(const double top_alpha,
                                                     const double bottom_alpha,
                                                     const ViewParameters& view_parameters,
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
          const double alpha = z_position * (top_alpha - bottom_alpha) + bottom_alpha;
          glColor4f(alpha, alpha, alpha, 1.0);
          
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

  // Wire frame animation.
  /*
  const double kMargin = 0.05;
  const double pivots[4] = { 1.0 / 8, 3.0 / 8, 5.0 / 8, 7.0 / 8};
  double diff = 1.0;
  for (int i = 0; i < 4; ++i)
    diff = min(diff, fabs(animation - pivots[i]));
  // const double kMargin = 0.05;
  // const double diff = min(fabs(animation - 0.25), fabs(animation - 0.75));
  if (diff < kMargin) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(2.0);
    const double alpha = min(1.0, 2.0 * (kMargin - diff) / kMargin);
    const Vector3i color(1.0, 1.0, 0.0);
    for (const auto& item : wire_frames) {
      const int room = item.first;
      const vector<vector<Vector3d> >& wire_frame = item.second;
      for (const auto& quad : wire_frame) {
        glBegin(GL_LINE_STRIP);
        glColor4f(color[0], color[1], color[2], alpha);
        for (int i = 0; i < 4; ++i) {
          const Vector3d point = view_parameters.TransformRoom(quad[i], room, -1, air_to_tree_progress, animation, max_vertical_shift);
          glVertex3d(point[0], point[1], point[2]);
        }
        glEnd();
      }
    }
    
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
  }
  */
  
  /*
  const double shrink_ratio = air_to_tree_progress * max_shrink_ratio + (1.0 - air_to_tree_progress);
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
      if (segment.type == Segment::CEILING) {
        continue;
      } else if (segment.type == Segment::DOOR) {
        continue;
      } else if (segment.type == Segment::FLOOR) {
        room = segment.floor_info;
      } else if (segment.type == Segment::WALL) {
        room = segment.wall_info[0];
      } else {
        cerr << "Invalid" << endl;
        exit (1);
      }
      const BoundingBox& bounding_box = view_parameters.GetIndoorPolygonDeformation().room_bounding_boxes[room];
      const Vector3d room_center = (bounding_box.min_xyz + bounding_box.max_xyz) / 2.0;

      for (const auto& triangle : segment.triangles) {
        if (triangle.image_index != texture)
          continue;

        for (int i = 0; i < 3; ++i) {
          glTexCoord2d(triangle.uvs[i][0], 1.0 - triangle.uvs[i][1]);

          const double z_position =
            max(0.0, min(1.0, (segment.vertices[triangle.indices[i]][2] - bottom_z) / (top_z - bottom_z)));
          const double alpha = z_position * (top_alpha - bottom_alpha) + bottom_alpha;
          glColor4f(alpha, alpha, alpha, 1.0);
          
          Vector3d global =
            indoor_polygon.ManhattanToGlobal(segment.vertices[triangle.indices[i]]);

          global = room_center + rotation * (global - room_center) * shrink_ratio;
          global += vertical_shift;
          
          glVertex3d(global[0], global[1], global[2]);
        }
      }
    }
    glEnd();
  }
  */
}
  
}  // namespace structured_indoor_modeling
