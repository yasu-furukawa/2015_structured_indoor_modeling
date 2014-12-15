#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../calibration/file_io.h"

using namespace std;

FloorplanRenderer::FloorplanRenderer() {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init(const std::string /*data_directory*/,
                             const Eigen::Matrix3d& /*floorplan_to_global_tmp*/) {
  /*
  file_io::FileIO file_io(data_directory);
  {
    ifstream ifstr;
    ifstr.open(file_io.GetFloorplan().c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  floorplan_to_global = floorplan_to_global_tmp;
  */
}

  /*
void FloorplanRenderer::RenderShape(const Shape& shape,
                                    const double floor_height,
                                    const PolygonStyle& style,
                                    const double alpha) {
  vector<Eigen::Vector3d> global_vertices(shape.vertices.size());
  for (int v = 0; v < (int)shape.vertices.size(); ++v) {
    global_vertices[v][0] = shape.vertices[v][0];
    global_vertices[v][1] = shape.vertices[v][1];
    global_vertices[v][2] = floor_height;
    global_vertices[v] = floorplan_to_global * global_vertices[v];
  }
  //glEnable(GL_BLEND);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBegin(GL_TRIANGLES);
  for (const auto& face : shape.faces) {
    // for (int i = 2; 0 <= i; --i) {
    for (int i = 0; i < 3; ++i) {
      glColor4f(style.fill_color[0],
                style.fill_color[1],
                style.fill_color[2],
                alpha);
      glVertex3d(global_vertices[face[i]][0],
                 global_vertices[face[i]][1],
                 global_vertices[face[i]][2]);
    }
  }
  glEnd();

  //glDisable(GL_BLEND);
}
  */

void FloorplanRenderer::Render(const FloorplanStyle& /*style*/, const double /*alpha*/) {
  /*
  for (const auto& component : floorplan.components) {
    RenderShape(component.outer_shape,
                floorplan.floor_height,
                style.outer_style,
                alpha);
    for (const auto& shape : component.inner_shapes) {
      RenderShape(shape,
                  floorplan.floor_height,
                  style.inner_style,
                  alpha);
    }
  }
  */
}
