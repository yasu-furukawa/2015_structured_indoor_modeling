#include <iostream>
#include "object_renderer.h"
#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer(const Floorplan& floorplan,
                               const IndoorPolygon& indoor_polygon)
  : floorplan(floorplan), indoor_polygon(indoor_polygon) {
  render = true;
}

ObjectRenderer::~ObjectRenderer() {
}

bool ObjectRenderer::Toggle() {
  render = !render;
  return render;
}

void ObjectRenderer::Init(const string data_directory) {
  FileIO file_io(data_directory);

  vertices.clear();
  colors.clear();

  vertices.resize(floorplan.GetNumRooms());
  colors.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    PointCloud point_cloud;
    point_cloud.Init(file_io.GetRefinedObjectClouds(room));

    vertices[room].resize(point_cloud.GetNumObjects());
    colors[room].resize(point_cloud.GetNumObjects());
    
    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Point& point = point_cloud.GetPoint(p);
      for (int i = 0; i < 3; ++i)
        vertices[room][point.object_id].push_back(point.position[i]);
      for (int i = 0; i < 3; ++i) {
        colors[room][point.object_id].push_back(point.color[i] / 255.0f);
      }
    }
  }

  vertices_org = vertices;
  colors_org = colors;

  ComputeBoundingBoxes();
}

void ObjectRenderer::InitGL() {
  initializeGLFunctions();
}
  
void ObjectRenderer::RenderAll(const double position) {
  if (!render)
    return;

  const bool kBlend = true; // ??? false

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  if (kBlend) {
    // glDisable(GL_DEPTH_TEST);
    glDepthMask(false);
    glEnable(GL_BLEND);
  }
  glEnable(GL_POINT_SMOOTH);

  const double kDurationPerObject = 0.4;
  
  for (int room = 0; room < (int)vertices.size(); ++room) {
    const double offset = (1.0 - kDurationPerObject) / max(1, (int)vertices[room].size() - 1);
    for (int object = 0; object < (int)vertices[room].size(); ++object) {
      // [object * offset, object * offset + kDurationPerObject].
      double scale;
      {
        const double start = object * offset;
        const double end = start + kDurationPerObject;
        if (position <= start || end <= position)
          scale = 1.0;
        else
          scale = sin(M_PI * (position - start) / kDurationPerObject) * 0.5 + 1.0;
      }
      
      for (int i = 0; i < (int)colors[room][object].size(); ++i) {
        colors[room][object][i] = min(1.0, scale * colors_org[room][object][i]);
      }
      
      glColorPointer(3, GL_FLOAT, 0, &colors[room][object][0]);
      glVertexPointer(3, GL_FLOAT, 0, &vertices[room][object][0]);

      if (kBlend) {
        glBlendColor(0, 0, 0, 0.5);
        //glBlendColor(0, 0, 0, 1.0);
        // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
        glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
      }
      glPointSize(1.0);
      
      glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object].size()) / 3);
    }
  }
  
	
  glDisable(GL_POINT_SMOOTH);
  if (kBlend) {
    glDisable(GL_BLEND);
    glDepthMask(true);
    // glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void ObjectRenderer::ComputeBoundingBoxes() {
  const int num_rooms = vertices.size();
  bounding_boxes.resize(num_rooms);
  for (int room = 0; room < num_rooms; ++room) {
    const int num_objects = vertices[room].size();
    bounding_boxes[room].resize(num_objects);
    for (int object = 0; object < num_objects; ++object) {

      Vector3d min_xyz, max_xyz;
      for (int p = 0; p < (int)vertices[room][object].size(); p += 3) {
        const Vector3d point(vertices[room][object][p + 0],
                             vertices[room][object][p + 1],
                             vertices[room][object][p + 2]);
        
        const Vector3d manhattan = indoor_polygon.GlobalToManhattan(point);
        if (p == 0) {
          min_xyz = max_xyz = manhattan;
        } else {
          for (int a = 0; a < 3; ++a) {
            min_xyz[a] = min(min_xyz[a], manhattan[a]);
            max_xyz[a] = max(max_xyz[a], manhattan[a]);
          }
        }
      }

      const double average_z = (min_xyz[2] + max_xyz[2]) / 2.0;
      BoundingBox& bounding_box = bounding_boxes[room][object];
      bounding_box.corners[0] =
        indoor_polygon.ManhattanToGlobal(Vector3d(min_xyz[0], min_xyz[1], average_z));
      bounding_box.corners[1] =
        indoor_polygon.ManhattanToGlobal(Vector3d(max_xyz[0], min_xyz[1], average_z));
      bounding_box.corners[2] =
        indoor_polygon.ManhattanToGlobal(Vector3d(max_xyz[0], max_xyz[1], average_z));
      bounding_box.corners[3] =
        indoor_polygon.ManhattanToGlobal(Vector3d(min_xyz[0], max_xyz[1], average_z));
    }
  }
}
  
void ObjectRenderer::RenderIcons(const double /* alpha */) {
  glEnable(GL_BLEND);
  
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glColor4f(0.8f, 1.0f, 0.8f, 0.5f);
  for (int room = 0; room < (int)bounding_boxes.size(); ++room) {
    for (const auto& bounding_box : bounding_boxes[room]) {
      for (int i = 0; i < 4; ++i)
        glVertex3d(bounding_box.corners[i][0],
                   bounding_box.corners[i][1],
                   bounding_box.corners[i][2]);
    }
  }
  glEnd();

  glLineWidth(0.5f);
  for (int room = 0; room < (int)bounding_boxes.size(); ++room) {
    for (const auto& bounding_box : bounding_boxes[room]) {
      glBegin(GL_LINE_LOOP);
      glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
      for (int i = 0; i < 4; ++i)
        glVertex3d(bounding_box.corners[i][0],
                   bounding_box.corners[i][1],
                   bounding_box.corners[i][2]);
      glEnd();
    }
  }
  
  glDisable(GL_BLEND);
}
  
}  // namespace structured_indoor_modeling
  
