#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../calibration/file_io.h"

using namespace std;

FloorplanRenderer::FloorplanRenderer() {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init(const std::string data_directory) {
  file_io::FileIO file_io(data_directory);
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  {
    ifstream ifstr;
    ifstr.open(file_io.GetRotationMat().c_str());

    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < 3; ++x) {
        ifstr >> rotation(y, x);
      }
    }
    ifstr.close();
  }
}

void FloorplanRenderer::RenderShape(const Shape& shape,
                                    const double floor_height,
                                    const Eigen::Matrix3d& rotation,
                                    const PolygonStyle& style) {
  vector<Eigen::Vector3d> global_vertices(shape.vertices.size());
  for (int v = 0; v < (int)shape.vertices.size(); ++v) {
    global_vertices[v][0] = shape.vertices[v][0];
    global_vertices[v][1] = shape.vertices[v][1];
    global_vertices[v][2] = floor_height;
    global_vertices[v] = rotation * global_vertices[v];
  }
  
  glBegin(GL_TRIANGLES);
  for (const auto& face : shape.faces) {
    // for (int i = 2; 0 <= i; --i) {
    for (int i = 0; i < 3; ++i) {
      glColor3f(style.fill_color[0],
                style.fill_color[1],
                style.fill_color[2]);
      glVertex3d(global_vertices[face[i]][0],
                 global_vertices[face[i]][1],
                 global_vertices[face[i]][2]);
    }
  }
  glEnd();
}

void FloorplanRenderer::Render(const FloorplanStyle& style) {
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  
  for (const auto& component : floorplan.components) {
    RenderShape(component.outer_shape,
                floorplan.floor_height,
                rotation,
                style.outer_style);
    for (const auto& shape : component.inner_shapes) {
      RenderShape(shape,
                  floorplan.floor_height,
                  rotation,
                  style.inner_style);
    }
  }
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}
