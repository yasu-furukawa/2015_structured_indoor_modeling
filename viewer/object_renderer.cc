#include <iostream>
#include "object_renderer.h"
#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/point_cloud.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
  render = false;
}

ObjectRenderer::~ObjectRenderer() {
  // glDeleteBuffers(2, vbos);
}

bool ObjectRenderer::Toggle() {
  render = !render;
  return render;
}

void ObjectRenderer::Init(const string data_directory) {
  /*
  FileIO file_io(data_directory);
  
  vertices.clear();
  colors.clear();
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    PointCloud point_cloud;
    point_cloud.Init(file_io.GetRefinedObjectClouds(room));

    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Point& point = point_cloud.GetPoint(p);
      for (int i = 0; i < 3; ++i)
        vertices.push_back(point.position[i]);
      for (int i = 0; i < 3; ++i) {
        colors.push_back(point.color[i] / 255.0f);
      }
    }
  }
  */

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
  /*
  glGenBuffers(1, vbos);

  //glEnableVertexAttribArray(0);
  //glEnableVertexAttribArray(1);
  
  glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof (float),
               &vertices[0], GL_STATIC_DRAW);

  // glBindBuffer(GL_ARRAY_BUFFER, vbos[1]);
  // glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof (float),
  // &colors[0], GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  */
}
  
void ObjectRenderer::RenderAll(const double /* alpha */) {
  if (!render)
    return;

  const bool kBlend = false; // ??? false

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  if (kBlend) {
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
  }
  glEnable(GL_POINT_SMOOTH);

  /*
  static int count = 0;
  ++count;
  const int kCycle = 40;
  count %= kCycle;
  */
  
  for (int room = 0; room < (int)vertices.size(); ++room) {
    for (int object = 0; object < (int)vertices[room].size(); ++object) {

      /*
      {
        double scale = 1.0;
        if (count < 20)
          scale = sin(2 * M_PI * count / kCycle) * 0.5 + 1.0;

        for (int i = 0; i < (int)colors[room][object].size(); ++i) {
          colors[room][object][i] = min(1.0, scale * colors_org[room][object][i]);
        }
      }
      */
      
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
    glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  



  /*
  glEnableVertexAttribArray(0);

  glEnableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, vbos[0]);
  glVertexPointer(3, GL_FLOAT, 0, 0);

  // glEnableClientState(GL_COLOR_ARRAY);
  // glBindBuffer(GL_ARRAY_BUFFER, vbos[1]);
  //glColorPointer(3, GL_FLOAT, 0, 0);
  //                 BUFFER_OFFSET(sizeof(GLfloat) * vertices.size()));

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPointSize(3.0);
  
  glDrawArrays(GL_POINTS, 0, vertices.size() / 3);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  // glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glEnable(GL_DEPTH_TEST);
  */

  
  /*
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPointSize(2.0);
  
  glBegin(GL_POINTS);
  for (int p = 0; p < colored_point_clouds[0][0].size(); ++p) {
    const ColoredPoint& colored_point = colored_point_clouds[0][0][p];
    
    glColor4f(colored_point.second[0], colored_point.second[1], colored_point.second[2], alpha / 2.0);
    glVertex3d(colored_point.first[0], colored_point.first[1], colored_point.first[2]);
  }  
  glEnd();

  glDisable(GL_BLEND);


  glClear(GL_DEPTH_BUFFER_BIT);

  glPointSize(1.0);
  glBegin(GL_POINTS);
  for (int p = 0; p < colored_point_clouds[0][0].size(); ++p) {
    const ColoredPoint& colored_point = colored_point_clouds[0][0][p];
    
    glColor4f(colored_point.second[0], colored_point.second[1], colored_point.second[2], alpha);
    glVertex3d(colored_point.first[0], colored_point.first[1], colored_point.first[2]);
  }  
  glEnd();
          */  
}

void ObjectRenderer::RenderRoom(const int room) const {

}

void ObjectRenderer::RenderObject(const int room, const int object) const {

}

}  // namespace structured_indoor_modeling
  
