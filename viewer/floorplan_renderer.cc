#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../base/file_io.h"
#include "../base/floorplan.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

FloorplanRenderer::FloorplanRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init() {
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

void FloorplanRenderer::Render(const double alpha) const {
  glBegin(GL_TRIANGLES);
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    glColor4f(0.3, 0.3, 0.3, alpha);
    const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
    for (const auto& triangle : floor_triangulation.triangles) {
      for (int i = 0; i < 3; ++i) {

        const int index = triangle.indices[i];
        const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
        glVertex3d(position[0], position[1], position[2]);
      }
    }
  }
  glEnd();

  glEnable(GL_LINE_SMOOTH);
  glLineWidth(5.0);
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    glBegin(GL_LINE_STRIP);
    glColor4f(0.3, 1.0, 1.0, alpha);
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const Vector3d position = floorplan.GetFloorVertexGlobal(room, vertex);
      glVertex3d(position[0], position[1], position[2]);
    }
    const int kFirst = 0;
    const Vector3d position = floorplan.GetFloorVertexGlobal(room, kFirst);
    glVertex3d(position[0], position[1], position[2]);
    glEnd();
  }
  

  

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

}  // namespace structured_indoor_modeling
