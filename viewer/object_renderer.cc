#include <iostream>
#include "object_renderer.h"
#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
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

}  // namespace structured_indoor_modeling
  
