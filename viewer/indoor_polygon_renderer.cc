#include <fstream>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

#include "../base/file_io.h"
#include "../base/indoor_polygon.h"
#include "indoor_polygon_renderer.h"
#include "tree_organizer.h"

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
                                                     const TreeOrganizer& tree_organizer,
                                                     const double air_to_tree_progress,
                                                     const double animation,
                                                     const double max_vertical_shift,
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

          global = tree_organizer.TransformRoom(global,
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
      const BoundingBox& bounding_box = tree_organizer.GetIndoorPolygonDeformation().room_bounding_boxes[room];
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
