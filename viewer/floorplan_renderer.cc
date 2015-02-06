#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../base/file_io.h"
#include "../base/floorplan.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

void DrawRectangleAndCircle(const Vector3d& position,
                            const Vector3d& next_position,
                            const Vector3d& z_axis,
                            const double radius,
                            const Vector4f& color) {
  const Vector3d x_axis = (next_position - position).normalized();
  const Vector3d y_axis = z_axis.cross(x_axis).normalized();

  const Vector3d v0 = position      - radius * y_axis;
  const Vector3d v1 = position      + radius * y_axis;
  const Vector3d v2 = next_position + radius * y_axis;
  const Vector3d v3 = next_position - radius * y_axis;

  glBegin(GL_QUADS);
  glColor4f(color[0], color[1], color[2], color[3]);
  glVertex3d(v3[0], v3[1], v3[2]);
  glVertex3d(v2[0], v2[1], v2[2]);
  glVertex3d(v1[0], v1[1], v1[2]);
  glVertex3d(v0[0], v0[1], v0[2]);
  glEnd();

  const int kNumSamples = 20;
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(color[0], color[1], color[2], color[3]);
  for (int i = 0; i < kNumSamples; ++i) {
    const double angle = 2.0 * M_PI * i / kNumSamples;
    Vector3d point = position + cos(angle) * radius * x_axis + sin(angle) * radius * y_axis;
    glVertex3d(point[0], point[1], point[2]);
  }
  glEnd();
}

}  // namespace

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
  glEnable(GL_SMOOTH);

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

  Vector3d z_axis(0, 0, 1);
  z_axis = floorplan.GetFloorplanToGlobal() * z_axis;
  
  const double kLineRadius = 2.0;
  const double radius = kLineRadius * floorplan.GetGridUnit();  

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const int next_vertex = (vertex + 1) % floorplan.GetNumRoomVertices(room);
      
      const Vector3d position      = floorplan.GetFloorVertexGlobal(room, vertex);
      const Vector3d next_position = floorplan.GetFloorVertexGlobal(room, next_vertex);

      DrawRectangleAndCircle(position, next_position, z_axis, radius, Vector4f(0.3, 1, 1, alpha));
    }
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
