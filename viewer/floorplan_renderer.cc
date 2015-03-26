#include "floorplan_renderer.h"

#include <iostream>
#include <fstream>

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"

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
  const PaintStyle kCorridorStyle(PaintStyle::VerticalStripe,
                                 Vector3f(216/255.0, 191/255.0, 216/255.0),
                                 Vector3f(1/255.0, 1/255.0, 1/255.0),
                                 2);

  const PaintStyle kDefaultStyle(PaintStyle::SolidColor,
                                 Vector3f(240/255.0, 240/255.0, 240/255.0),
                                 Vector3f(1/255.0, 1/255.0, 1/255.0),
                                 2);
  
  const PaintStyle kTileStyle(PaintStyle::Tile,
                              Vector3f(176/255.0, 196/255.0, 222/255.0),
                              Vector3f(1/255.0, 1/255.0, 1/255.0),
                              2);
  
  /*
  const PaintStyle kKitchenStyle(PaintStyle::Kitchen,
                                 Vector3f(216/255.0, 191/255.0, 216/255.0),
                                 Vector3f(1/255.0, 1/255.0, 1/255.0),
                                 2);
  */
  const PaintStyle kKitchenStyle(PaintStyle::SolidColor,
                                 Vector3f(255/255.0, 239/255.0, 214/255.0),
                                 Vector3f(1/255.0, 1/255.0, 1/255.0),
                                 2);
  
  const PaintStyle kDiningStyle(PaintStyle::Kitchen,
                                Vector3f(1, 1, 1),
                                Vector3f(1/255.0, 1/255.0, 1/255.0),
                                2);
  /*
  const PaintStyle kBedStyle(PaintStyle::Sheep,
                             Vector3f(1, 1, 1),
                             Vector3f(1/255.0, 1/255.0, 1/255.0),
                             2);
  */
  const PaintStyle kBedStyle(PaintStyle::SolidColor,
                             Vector3f(224/255.0, 239/255.0, 255/255.0),
                             Vector3f(1/255.0, 1/255.0, 1/255.0),
                             2);
  
  
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

FloorplanRenderer::FloorplanRenderer(const Floorplan& floorplan,
                                     const IndoorPolygon& indoor_polygon) 
  : floorplan(floorplan), indoor_polygon(indoor_polygon) {
  sheep_texture_id = -1;
  kitchen_texture_id = -1;
  tile_texture_id = -1;
}

FloorplanRenderer::~FloorplanRenderer() {
  if (sheep_texture_id != -1)
    widget->deleteTexture(sheep_texture_id);
  if (kitchen_texture_id != -1)
    widget->deleteTexture(kitchen_texture_id);
  if (tile_texture_id != -1)
    widget->deleteTexture(tile_texture_id);
}

void FloorplanRenderer::Init() {
}

void FloorplanRenderer::InitGL(QGLWidget* widget_tmp) {
  widget = widget_tmp;
  
  initializeGLFunctions();
  // Likely need to change for windows.
  {
	#ifdef _WIN32
//    TCHAR szBuf[MAX_PATH];
//    GetCurrentDirectory(_countof(szBuf), szBuf);
//    OutputDebugString(szBuf);
    sheep_image.load("../viewer/texture/sheep.png");

	#else
    sheep_image.load("texture/sheep.png");
	#endif
    if (sheep_image.isNull()) {
      cout << "texture/sheep.png cannot be loaded." << endl
           << "Likely using visual studio. Need to change a relative path infloorplan_renderer.cc." << endl;
      exit (1);
    }
    sheep_texture_id = widget->bindTexture(sheep_image);
  }
  {
    #ifdef _WIN32
    kitchen_image.load("../viewer/texture/kitchen.jpg");
    #else
    kitchen_image.load("texture/kitchen.jpg");
    #endif

    if (kitchen_image.isNull()) {
      cout << "texture/kitchen.jpg cannot be loaded." << endl
           << "Likely using visual studio. Need to change a relative path infloorplan_renderer.cc." << endl;
      exit (1);
    }
    kitchen_texture_id = widget->bindTexture(kitchen_image);
  }
  {
    #ifdef _WIN32
    tile_image.load("../viewer/texture/tile4.jpg");
    #else
    tile_image.load("texture/tile4.jpg");
    #endif

    if (tile_image.isNull()) {
      cout << "texture/tile4.jpg cannot be loaded." << endl
           << "Likely using visual studio. Need to change a relative path infloorplan_renderer.cc." << endl;
      exit (1);
    }
    tile_texture_id = widget->bindTexture(tile_image);
  }
  
  // Set nearest filtering mode for texture minification
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  
  // Set bilinear filtering mode for texture magnification
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  // Wrap texture coordinates by repeating
  // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}
  
PaintStyle FloorplanRenderer::GetPaintStyle(const vector<string>& room_names) const {
  for (const auto& word : room_names) {
    if (word == "shower" || word == "showerroom" || word == "bathroom")
      return kTileStyle;
    else if (word == "kitchen")
      return kKitchenStyle;
    else if (word == "dining")
      return kDiningStyle;
    else if (word == "bedroom")
      return kBedStyle;
    else if (word == "corridor")
      return kCorridorStyle;
  }
  return kDefaultStyle;
}

void FloorplanRenderer::RenderLabels() {
  glDisable(GL_BLEND);
  glBegin(GL_TRIANGLES);
  glDisable(GL_CULL_FACE);
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const FloorCeilingTriangulation floor_triangulation = floorplan.GetFloorTriangulation(room);
    for (const auto& triangle : floor_triangulation.triangles) {
      for (int i = 0; i < 3; ++i) {
        const int index = triangle.indices[i];
        const Vector3d position = floorplan.GetFloorVertexGlobal(room, index);
        glColor3ub(0, 0, room + 1);
        glVertex3d(position[0], position[1], position[2]);
      }
    }
  }
  glEnd();
  glEnable(GL_BLEND);
}
  
void FloorplanRenderer::Render(const double alpha,
                               const GLint viewport_tmp[],
                               const GLdouble modelview_tmp[],
                               const GLdouble projection_tmp[],
                               const bool emphasize,
                               const double height_adjustment) {
  viewport = viewport_tmp;
  modelview = modelview_tmp;
  projection = projection_tmp;
  
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

  // Indoorpolygon outline.
  // Looked very bad.
  // RenderIndoorPolygonOutline(alpha);
  
  // Floorplan outline.
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const PaintStyle paint_style = GetPaintStyle(floorplan.GetRoomName(room));
    RenderFloorplanOutline(room, paint_style, alpha, emphasize, height_adjustment);
  }

  for (int door = 0; door < floorplan.GetNumDoors(); ++door) {
    {
      const Vector3d start = floorplan.GetDoorVertexGlobal(door, 1);
      const Vector3d end   = floorplan.GetDoorVertexGlobal(door, 0);
      const Vector3d top   = floorplan.GetDoorVertexGlobal(door, 2);
      RenderDoor(start, end, top, emphasize, height_adjustment);
    }
    {
      const Vector3d start = floorplan.GetDoorVertexGlobal(door, 5);
      const Vector3d end   = floorplan.GetDoorVertexGlobal(door, 4);
      const Vector3d top   = floorplan.GetDoorVertexGlobal(door, 6);
      // RenderDoor(start, end, top, emphasize, height_adjustment);
      // No animation.
      RenderDoor(start, end, top, emphasize, 0.0);
    }
  }
}

void FloorplanRenderer::RenderDoor(const Vector3d& start,
                                   const Vector3d& end,
                                   const Vector3d& top,
                                   const bool emphasize,
                                   const double height_adjustment) const {
  // Draw arch and the door at 90 degrees.
  Vector3d door_x = end - start;
  Vector3d zaxis = (top - start).normalized();
  Vector3d door_y = zaxis.cross(door_x);

  double max_angle;
  if (emphasize)
    max_angle = 80.0 * M_PI / 180.0 * height_adjustment;
  else
    max_angle = 0.0;
  const double kAngleStep = 5.0 * M_PI / 180.0;
  // Rotate door around axis by angle.

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  {
    glLineWidth(0.5);
    glBegin(GL_LINE_STRIP);
    glColor4ub(50, 205, 50, 255);
    glVertex3d(start[0], start[1], start[2]);
    glVertex3d(end[0], end[1], end[2]);
    for (double angle = kAngleStep; angle <= max_angle; angle += kAngleStep) {
      Vector3d pos = cos(angle) * door_x + sin(angle) * door_y;
      pos += start;
      glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
  }
  {
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor4ub(50, 205, 50, 255);
    glVertex3d(start[0], start[1], start[2]);
    Vector3d pos = cos(max_angle) * door_x + sin(max_angle) * door_y;
    pos += start;
    glVertex3d(pos[0], pos[1], pos[2]);
    glEnd();
  }

  glBegin(GL_TRIANGLE_FAN);
  glColor4ub(50, 205, 50, 50);
  glVertex3d(start[0], start[1], start[2]);
  glVertex3d(end[0], end[1], end[2]);
  for (double angle = kAngleStep; angle <= max_angle; angle += kAngleStep) {
    Vector3d pos = cos(angle) * door_x + sin(angle) * door_y;
    pos += start;
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
  glDisable(GL_BLEND);
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
      RenderSolidColor(room, paint_style, unit, alpha);
      break;
    }
    case PaintStyle::VerticalStripe: {
      RenderVerticalStripe(room, paint_style, unit, alpha);
      break;
    }
    case PaintStyle::WaterDrop: {
      RenderWaterDrop(room, paint_style, unit, alpha);
      break;
    }
    case PaintStyle::Sheep: {
      RenderSheep(room, paint_style, unit, alpha);
      break;
    }
    case PaintStyle::Kitchen: {
      RenderKitchen(room, paint_style, unit, alpha);
      break;
    }
    case PaintStyle::Tile: {
      RenderTile(room, paint_style, unit, alpha);
      break;
    }
    }    
  }
}    

void FloorplanRenderer::RenderSolidColor(const int room,
                                         const PaintStyle& paint_style,
                                         const double /* unit */,
                                         const double alpha) const {
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
}

void FloorplanRenderer::RenderVerticalStripe(const int room,
                                             const PaintStyle& paint_style,
                                             const double unit,
                                             const double /* alpha */) const {
  Vector2d x_range, y_range;
  ComputeRanges(floorplan, room, &x_range, &y_range);

  const double spacing = unit * 2;
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
}

void FloorplanRenderer::RenderWaterDrop(const int room,
                                        const PaintStyle& paint_style,
                                        const double unit,
                                        const double /* alpha */) const {
  Vector2d x_range, y_range;
  ComputeRanges(floorplan, room, &x_range, &y_range);
  
  const double spacing = unit * 2;
  const double radius = unit * 0.5;
  const double margin = spacing / 2.0;
  
  for (double y = y_range[0] - margin; y < y_range[1] + spacing + margin; y += spacing) {
    for (double x = x_range[0] - margin; x < x_range[1] + spacing + margin; x += spacing) {
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
}

void FloorplanRenderer::RenderSheep(const int room,
                                    const PaintStyle& paint_style,
                                    const double unit,
                                    const double alpha) const {
  const double kSheepTextureScale = 30;
  RenderTexture(room, paint_style, unit, alpha, sheep_texture_id, kSheepTextureScale);
}

void FloorplanRenderer::RenderKitchen(const int room,
                                      const PaintStyle& paint_style,
                                        const double unit,
                                      const double alpha) const {
  const double kKitchenTextureScale = 30;
  RenderTexture(room, paint_style, unit, alpha, kitchen_texture_id, kKitchenTextureScale);
}

void FloorplanRenderer::RenderTile(const int room,
                                   const PaintStyle& paint_style,
                                   const double unit,
                                   const double alpha) const {
  const double kTileTextureScale = 30;
  RenderTexture(room, paint_style, unit, alpha, tile_texture_id, kTileTextureScale);
}
  
void FloorplanRenderer::RenderTexture(const int room,
                                      const PaintStyle& /* paint_style */,
                                      const double unit,
                                      const double alpha,
                                      const GLint texture_id,
                                      const double texture_scale) const {
  Vector3d screen_x_axis;
  {
    Vector3d center, right;
    gluUnProject(viewport[1] / 2, viewport[3] / 2, 1, modelview, projection, viewport,
                 &center[0], &center[1], &center[2]);
    gluUnProject(viewport[1] / 2 + 1, viewport[3] / 2, 1, modelview, projection, viewport,
                 &right[0], &right[1], &right[2]);
    screen_x_axis = (right - center).normalized();
  }
  
  Vector2d x_range, y_range;
  ComputeRanges(floorplan, room, &x_range, &y_range);
  // Make it bigger and square for easy handling of rotation.
  if (y_range[1] - y_range[0] > x_range[1] - x_range[0]) {
    x_range[1] = x_range[0] + (y_range[1] - y_range[0]);
  } else {
    y_range[1] = y_range[0] + (x_range[1] - x_range[0]);
  }
  
  const double spacing = unit * texture_scale;
  const double margin = spacing / 2.0;
  
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, alpha);
  
  const Vector3d local00(x_range[0] - margin, y_range[0] - margin, floorplan.GetFloorHeight(room));
  const Vector3d local01(x_range[0] - margin, y_range[1] + margin, floorplan.GetFloorHeight(room));
  const Vector3d local10(x_range[1] + margin, y_range[0] - margin, floorplan.GetFloorHeight(room));
  const Vector3d local11(x_range[1] + margin, y_range[1] + margin, floorplan.GetFloorHeight(room));
  const Vector3d global00 = floorplan.GetFloorplanToGlobal() * local00;
  const Vector3d global01 = floorplan.GetFloorplanToGlobal() * local01;
  const Vector3d global10 = floorplan.GetFloorplanToGlobal() * local10;
  const Vector3d global11 = floorplan.GetFloorplanToGlobal() * local11;

  vector<Vector3d> globals;
  globals.push_back(global00);
  globals.push_back(global10);
  globals.push_back(global11);
  globals.push_back(global01);

  vector<Vector2d> uvs;
  uvs.push_back(Vector2d(local00[0] / spacing, local00[1] / spacing));
  uvs.push_back(Vector2d(local10[0] / spacing, local10[1] / spacing));
  uvs.push_back(Vector2d(local11[0] / spacing, local11[1] / spacing));
  uvs.push_back(Vector2d(local01[0] / spacing, local01[1] / spacing));

  const Vector3d x_axis = (global10 - global00).normalized();
  const Vector3d y_axis = (global01 - global00).normalized();
  const double x_dot = screen_x_axis.dot(x_axis);
  const double y_dot = screen_x_axis.dot(y_axis);
  // Offset;
  int uv_offset = 0;
  if (fabs(x_dot) > fabs(y_dot)) {
    if (x_dot > 0.0) {
      uv_offset = 0;
    } else {
      uv_offset = 2;
    }
  } else {
    if (y_dot > 0.0) {
      uv_offset = 3;
    } else {
      uv_offset = 1;
    }
  }

  for (int i = 0; i < 4; ++i) {
    glTexCoord2d(uvs[(i + uv_offset) % 4][0], uvs[(i + uv_offset) % 4][1]);
    glVertex3d(globals[i][0], globals[i][1], globals[i][2]);
  }
  
  glEnd();
  glDisable(GL_TEXTURE_2D);
}
  
void FloorplanRenderer::RenderFloorplanOutline(const int room,
                                               const PaintStyle& paint_style,
                                               const double alpha,
                                               const bool /* emphasize */,
                                               const double /* height_adjustment */) const {
  Vector3d z_axis(0, 0, 1);
  z_axis = floorplan.GetFloorplanToGlobal() * z_axis;
  double radius = paint_style.stroke_width * floorplan.GetGridUnit();
  Vector3d color(paint_style.stroke_color[0],
                 paint_style.stroke_color[1],
                 paint_style.stroke_color[2]);

  // Boundary.
  for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
    const int next_vertex = (vertex + 1) % floorplan.GetNumRoomVertices(room);
    
    const Vector3d position      = floorplan.GetFloorVertexGlobal(room, vertex);
    const Vector3d next_position = floorplan.GetFloorVertexGlobal(room, next_vertex);
    
    DrawRectangleAndCircle(position, next_position, z_axis, radius,
                           Vector4f(color[0], color[1], color[2], alpha));
  }
}

void FloorplanRenderer::RenderIndoorPolygonOutline(const double alpha) const {
  Vector3d z_axis(0, 0, 1);
  z_axis = floorplan.GetFloorplanToGlobal() * z_axis;
  // const double kStrokeWidth = 1;
  // double radius = kStrokeWidth * floorplan.GetGridUnit();
  Vector3d kColor(0.4f, 0.4f, 0.4f);
  // paint_style.stroke_color...


  glBegin(GL_LINES);
  glColor4f(kColor[0], kColor[1], kColor[2], alpha);
  for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
    const Segment& segment = indoor_polygon.GetSegment(s);
    if (segment.type != Segment::WALL)
      continue;

    for (const auto& triangle : segment.triangles) {
      const Vector3d vs[3] = { segment.vertices[triangle.indices[0]],
                               segment.vertices[triangle.indices[1]],
                               segment.vertices[triangle.indices[2]] };

      Vector3d gs[3];
      for (int i = 0; i < 3; ++i)
        gs[i] = indoor_polygon.ManhattanToGlobal(vs[i]);

      for (int i = 0; i < 3; ++i) {
        const int next_i = (i + 1) % 3;
        if (ShouldRender(vs[i], vs[next_i])) {

          glVertex3f(gs[i][0], gs[i][1], gs[i][2]);
          glVertex3f(gs[next_i][0], gs[next_i][1], gs[next_i][2]);
        }
      }
    }
  }
  glEnd();
}

bool FloorplanRenderer::ShouldRender(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
  if (lhs[2] != rhs[2])
    return false;

  if (lhs[0] == rhs[0] || lhs[1] == rhs[1])
    return true;
  else
    return false;
}
  
}  // namespace structured_indoor_modeling
