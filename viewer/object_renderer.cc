#include <iostream>
#include "object_renderer.h"
#include "../calibration/file_io.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer() {
}

ObjectRenderer::~ObjectRenderer() {

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
}

void ObjectRenderer::RenderAll(const double alpha) {
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
  
}

void ObjectRenderer::RenderRoom(const int room) {

}

void ObjectRenderer::RenderObject(const int room, const int object) {

}

}  // namespace structured_indoor_modeling
  
