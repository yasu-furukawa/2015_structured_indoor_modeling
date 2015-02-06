#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../base/file_io.h"
#include "../base/floorplan.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const PaintStyle kDefaultStyle(Vector3f(0.3, 0.3, 0.3),
                               Vector3f(0.3, 1.0, 0.3),
                               3.0);

const PaintStyle kShowerStyle(Vector3f(0.3, 1.0, 1.0),
                              Vector3f(0.3, 0.3, 0.3),
                              3.0);

const PaintStyle kKitchenStyle(Vector3f(1.0, 0.8, 0.2),
                               Vector3f(0.3, 0.3, 0.3),
                               3.0);

const PaintStyle kDiningStyle(Vector3f(1.0, 0.8, 0.2),
                              Vector3f(0.3, 0.3, 0.3),
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

    // Boundary.
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(paint_style.stroke_width);
    glBegin(GL_LINE_STRIP);
    glColor4f(paint_style.stroke_color[0],
              paint_style.stroke_color[1],
              paint_style.stroke_color[2],
              alpha);
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const Vector3d position = floorplan.GetFloorVertexGlobal(room, vertex);
      glVertex3d(position[0], position[1], position[2]);
    }
    const int kFirst = 0;
    const Vector3d position = floorplan.GetFloorVertexGlobal(room, kFirst);
    glVertex3d(position[0], position[1], position[2]);
    glEnd();
  }
}

}  // namespace structured_indoor_modeling
