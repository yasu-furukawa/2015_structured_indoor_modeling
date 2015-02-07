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

  glEnable(GL_POLYGON_SMOOTH);

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

// =======
const PaintStyle kDefaultStyle(Vector3f(0.3, 0.3, 0.3),
                               Vector3f(0.3, 1.0, 0.3),
                               3.0);

const PaintStyle kShowerStyle(Vector3f(0.3, 0.3, 0.3),
                              Vector3f(0.3, 1.0, 1.0),
                              3.0);

const PaintStyle kKitchenStyle(Vector3f(0.3, 0.3, 0.3),
                               Vector3f(1.0, 0.8, 0.2),
                               3.0);

const PaintStyle kDiningStyle(Vector3f(0.3, 0.3, 0.3),
                              Vector3f(1.0, 0.8, 0.2),
                              3.0);
  
FloorplanRenderer::FloorplanRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init() {
}

PaintStyle FloorplanRenderer::GetPaintStyle(const vector<string>& room_names) const {
  for (const auto& word : room_names) {
    if (word == "showerroom" || word == "bathroom")
      return kShowerStyle;
    else if (word == "kitchen")
      return kKitchenStyle;
    else if (word == "dining")
      return kDiningStyle;
  }
  return kDefaultStyle;
}

void FloorplanRenderer::Render(const double alpha) const {
  // Interior.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const PaintStyle paint_style =
      GetPaintStyle(floorplan.GetRoomName(room));

    glBegin(GL_TRIANGLES);
    glColor4f(paint_style.fill_color[0],
              paint_style.fill_color[1],
              paint_style.fill_color[2],
              alpha);
    const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
    for (const auto& triangle : floor_triangulation.triangles) {
      for (int i = 0; i < 3; ++i) {

        const int index = triangle.indices[i];
        const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
        glVertex3d(position[0], position[1], position[2]);
      }
    }
    glEnd();

    Vector3d z_axis(0, 0, 1);
    z_axis = floorplan.GetFloorplanToGlobal() * z_axis;
  
    const double radius = paint_style.stroke_width * floorplan.GetGridUnit();  

    // Boundary.
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const int next_vertex = (vertex + 1) % floorplan.GetNumRoomVertices(room);
      
      const Vector3d position      = floorplan.GetFloorVertexGlobal(room, vertex);
      const Vector3d next_position = floorplan.GetFloorVertexGlobal(room, next_vertex);

      DrawRectangleAndCircle(position, next_position, z_axis, radius,
                             Vector4f(paint_style.stroke_color[0],
                                      paint_style.stroke_color[1],
                                      paint_style.stroke_color[2],
                                      alpha));
    }
  }  
}

}  // namespace structured_indoor_modeling
