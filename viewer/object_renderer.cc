#include <iostream>
#include "object_renderer.h"
#include "../base/file_io.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer() {
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
  FileIO file_io(data_directory);

  colored_point_clouds.resize(1);
  colored_point_clouds[0].resize(1);

  /*
  char buffer[1024];
  sprintf(buffer, "%s/object_cloud.ply", data_directory.c_str());
  ifstream ifstr;
  // ifstr.open("object_cloud.ply");
  ifstr.open(buffer);
  string header;
  for (int i = 0; i < 6; ++i)
    ifstr >> header;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 37; ++i)
    ifstr >> header;

  colored_point_clouds[0][0].resize(num_points);
  for (int p = 0; p < num_points; ++p) {
    double dtmp;
    for (int i = 0; i < 2; ++i)
      ifstr >> dtmp;
    for (int i = 0; i < 3; ++i)
      ifstr >> colored_point_clouds[0][0][p].first[i];
    for (int i = 0; i < 3; ++i) {
      ifstr >> colored_point_clouds[0][0][p].second[i];
      colored_point_clouds[0][0][p].second[i] /= 255.0;
    }
    for (int i = 0; i < 4; ++i)
      ifstr >> dtmp;
  }
  ifstr.close();
  */

  char buffer[1024];
  sprintf(buffer, "%s/object_cloud2.ply", data_directory.c_str());
  ifstream ifstr;
  // ifstr.open("object_cloud.ply");
  ifstr.open(buffer);
  string header;
  for (int i = 0; i < 6; ++i)
    ifstr >> header;
  int num_points;
  ifstr >> num_points;
  for (int i = 0; i < 22; ++i)
    ifstr >> header;

  colored_point_clouds[0][0].resize(num_points);
  for (int p = 0; p < num_points; ++p) {
    for (int i = 0; i < 3; ++i)
      ifstr >> colored_point_clouds[0][0][p].first[i];
    for (int i = 0; i < 3; ++i) {
      ifstr >> colored_point_clouds[0][0][p].second[i];
      colored_point_clouds[0][0][p].second[i] /= 255.0;
    }
    double dtmp;
    ifstr >> dtmp;
  }
  ifstr.close();


  vertices.clear();
  colors.clear();

  for (int p = 0; p < num_points; ++p) {
    for (int i = 0; i < 3; ++i)
      vertices.push_back(colored_point_clouds[0][0][p].first[i]);
    for (int i = 0; i < 3; ++i) {
      colors.push_back(colored_point_clouds[0][0][p].second[i]);
    }
  }
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

void ObjectRenderer::RenderAll(const double alpha) {
  if (!render)
    return;

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  // glDisable(GL_DEPTH_TEST);
  //glEnable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
	

  glColorPointer(3, GL_FLOAT, 0, &colors[0]);
  glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);

  // glBlendColor(0, 0, 0, 0.5);
  //glBlendColor(0, 0, 0, 1.0);
  //glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
  glPointSize(3.0);

  glDrawArrays(GL_POINTS, 0, ((int)vertices.size()) / 3);
	
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
  glDisable(GL_POINT_SMOOTH);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  

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

void ObjectRenderer::RenderRoom(const int room) {

}

void ObjectRenderer::RenderObject(const int room, const int object) {

}

}  // namespace structured_indoor_modeling
  
