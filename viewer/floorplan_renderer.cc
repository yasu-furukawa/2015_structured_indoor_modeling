#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../base/file_io.h"
#include "../base/floorplan.h"

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <windows.h>
#include <GL/glu.h>
//#ifndef __glew_h__
//#include <GL/glew.h>
//#include <GL/glext.h>
//#endif
#else
#include <OpenGL/glu.h>
#endif

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

// =======
  const PaintStyle kDefaultStyle(PaintStyle::SolidColor,
                                 Vector3f(1.0, 0.98, 0.70),
                                 Vector3f(0, 0, 0),
                                 1.5);

  const PaintStyle kShowerStyle(PaintStyle::WaterDrop,
                              Vector3f(176/255.0, 196/255.0, 222/255.0),
                              Vector3f(0, 0, 0),
                              1.5);

  const PaintStyle kKitchenStyle(PaintStyle::VerticalStripe,
                                 Vector3f(216/255.0, 191/255.0, 216/255.0),
                                 Vector3f(0, 0, 0),
                                 1.5);

  const PaintStyle kDiningStyle(PaintStyle::SolidColor,
                              Vector3f(1, 1, 1),
                              Vector3f(0, 0, 0),
                              1.5);

  const PaintStyle kBedStyle(PaintStyle::Sheep,
                             Vector3f(1, 1, 1),
                             Vector3f(0, 0, 0),
                             1.5);
  
  
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

void ComputeRanges(const Floorplan& floorplan, const int room, Vector2d* x_range, Vector2d* y_range) {
  const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
  bool first = true;
  for (const auto& triangle : floor_triangulation.triangles) {
    for (int i = 0; i < 3; ++i) {
      const int index = triangle.indices[i];
      const Vector2d position = floorplan.GetRoomVertexLocal(room, index);
      
      if (first) {
        (*x_range)[0] = (*x_range)[1] = position[0];
        (*y_range)[0] = (*y_range)[1] = position[1];
        first = false;
      } else {
        (*x_range)[0] = min((*x_range)[0], position[0]);
        (*x_range)[1] = max((*x_range)[1], position[0]);
        (*y_range)[0] = min((*y_range)[0], position[1]);
        (*y_range)[1] = max((*y_range)[1], position[1]);
      }
    }
  }
}
  
}  // namespace

FloorplanRenderer::FloorplanRenderer(const Floorplan& floorplan) : floorplan(floorplan) {
}

FloorplanRenderer::~FloorplanRenderer() {
}

void FloorplanRenderer::Init() {
}

PaintStyle FloorplanRenderer::GetPaintStyle(const vector<string>& room_names) const {
  for (const auto& word : room_names) {
    if (word == "shower" || word == "showerroom" || word == "bathroom")
      return kShowerStyle;
    else if (word == "kitchen")
      return kKitchenStyle;
    else if (word == "dining")
      return kDiningStyle;
  }
  return kDefaultStyle;
}

void FloorplanRenderer::Render(const double alpha) const {
  Vector2d x_range, y_range;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const Vector2d center = floorplan.GetRoomCenterLocal(room);
    if (room == 0) {
      x_range[0] = x_range[1] = center[0];
      y_range[0] = y_range[1] = center[1];
    } else {
      x_range[0] = min(x_range[0], center[0]);
      x_range[1] = max(x_range[1], center[0]);
      y_range[0] = min(y_range[0], center[1]);
      y_range[1] = max(y_range[1], center[1]);
    }
  }

  const double unit = max(x_range[1] - x_range[0], y_range[1] - y_range[0]) / 100;

  // Interior.
  glEnable(GL_STENCIL_TEST);  
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const PaintStyle paint_style = GetPaintStyle(floorplan.GetRoomName(room));
    const bool kSetStencil = true;
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilMask(0xFF);
    //glDepthMask(GL_FALSE);
    glClear(GL_STENCIL_BUFFER_BIT);
    RenderRoomFill(room, unit, paint_style, alpha, kSetStencil);
    
    const bool kUseStencil = false;
    glStencilFunc(GL_EQUAL, 1, 0xFF);
    glStencilMask(0x00);
    //glDepthMask(GL_TRUE);
    RenderRoomFill(room, unit, paint_style, alpha, kUseStencil);
  }
  glDisable(GL_STENCIL_TEST);

  // Outline.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const PaintStyle paint_style = GetPaintStyle(floorplan.GetRoomName(room));
    RenderRoomStroke(room, paint_style, alpha);
  }
}

void FloorplanRenderer::RenderRoomFill(const int room,
                                       const double unit,
                                       const PaintStyle& paint_style,
                                       const double alpha,
                                       const bool set_stencil) const {
  if (set_stencil) {
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glBegin(GL_TRIANGLES);
    const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    for (const auto& triangle : floor_triangulation.triangles) {
      for (int i = 0; i < 3; ++i) {
        
        const int index = triangle.indices[i];
        const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
        glVertex3d(position[0], position[1], position[2]);
      }
    }
    glEnd();
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  } else {
    switch (paint_style.fill_style) {
    case PaintStyle::SolidColor: {
      // Avoid strange weak lines between triangle.
      glDisable(GL_BLEND);
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
      glEnable(GL_BLEND);
      break;
    }
    case PaintStyle::VerticalStripe: {
      Vector2d x_range, y_range;
      ComputeRanges(floorplan, room, &x_range, &y_range);

      const double spacing = unit * 3;
      glLineWidth(2.0);
      glBegin(GL_LINES);
      for (double x = x_range[0]; x < x_range[1]; x += spacing) {
        const Vector3d top_local(x, y_range[0], floorplan.GetFloorHeight(room));
        const Vector3d bottom_local(x, y_range[1], floorplan.GetFloorHeight(room));
        const Vector3d top_global = floorplan.GetFloorplanToGlobal() * top_local;
        const Vector3d bottom_global = floorplan.GetFloorplanToGlobal() * bottom_local;
        glColor3f(paint_style.fill_color[0], paint_style.fill_color[1], paint_style.fill_color[2]);
        glVertex3d(top_global[0], top_global[1], top_global[2]);
        glVertex3d(bottom_global[0], bottom_global[1], bottom_global[2]);
      }
      for (double y = y_range[0]; y < y_range[1]; y += spacing) {
        const Vector3d top_local(x_range[0], y, floorplan.GetFloorHeight(room));
        const Vector3d bottom_local(x_range[1], y, floorplan.GetFloorHeight(room));
        const Vector3d top_global = floorplan.GetFloorplanToGlobal() * top_local;
        const Vector3d bottom_global = floorplan.GetFloorplanToGlobal() * bottom_local;
        glColor3f(paint_style.fill_color[0], paint_style.fill_color[1], paint_style.fill_color[2]);
        glVertex3d(top_global[0], top_global[1], top_global[2]);
        glVertex3d(bottom_global[0], bottom_global[1], bottom_global[2]);
      }
      glEnd();
      break;
    }
    case PaintStyle::WaterDrop: {
      Vector2d x_range, y_range;
      ComputeRanges(floorplan, room, &x_range, &y_range);

      const double spacing = unit * 2;
      const double radius = unit * 0.5;

      for (double y = y_range[0] - spacing / 2.0; y < y_range[1] + spacing / 2.0; y += spacing) {
        for (double x = x_range[0] - spacing / 2.0; x < x_range[1] + spacing / 2.0; x += spacing) {
          const int kNumSamples = 20;
          glBegin(GL_TRIANGLE_FAN);
          glColor3f(paint_style.fill_color[0], paint_style.fill_color[1], paint_style.fill_color[2]);
          for (int i = 0; i < kNumSamples; ++i) {
            const double angle = 2.0 * M_PI * i / kNumSamples;
            
            const Vector3d local(x + cos(angle) * radius,
                                 y + sin(angle) * radius,
                                 floorplan.GetFloorHeight(room));
            const Vector3d global = floorplan.GetFloorplanToGlobal() * local;

            glVertex3d(global[0], global[1], global[2]);
          }
          glEnd();
        }
      }
      break;
    }
    case PaintStyle::Sheep: {
      
      
    }
    }    
  }

  
}    

void FloorplanRenderer::RenderRoomStroke(const int room,
                                         const PaintStyle& paint_style,
                                         const double alpha) const {
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
  
}  // namespace structured_indoor_modeling
