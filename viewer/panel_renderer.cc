#include <iostream>
#include <OpenGL/glu.h>

#include "../calibration/file_io.h"
#include "main_widget.h"
#include "panel_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const double PanelRenderer::kWidthRatio = 0.2;
const int PanelRenderer::kTextHeight = 6;
const int PanelRenderer::kFrameMargin = 5;

PanelRenderer::PanelRenderer(const PolygonRenderer& polygon_renderer,
                             const GLint* viewport) :
  polygon_renderer(polygon_renderer), viewport(viewport) {
  thumbnail_texid = 0;
}

PanelRenderer::~PanelRenderer() {
  glDeleteTextures(1, &thumbnail_texid);
}

void PanelRenderer::Init(const std::string& data_directory) {
  FileIO file_io(data_directory);

  const int room_num = polygon_renderer.GetFloorplan().GetNumRooms();
  room_thumbnails.resize(room_num);
  for (int room = 0; room < room_num; ++room) {
    room_thumbnails[room].load(file_io.GetRoomThumbnail(room).c_str());
  }
}

void PanelRenderer::InitGL(MainWidget* /*main_widget*/) {
  glGenTextures(1, &thumbnail_texid);
}

void PanelRenderer::RenderThumbnail(const double alpha,
                                    const int room_highlighted,
                                    const Vector2i& render_pos,
                                    const Vector3d& color,
                                    const double scale,
                                    MainWidget* main_widget) {
  if (room_highlighted == -1) {
    return;
  }
  const vector<string>& name = polygon_renderer.GetFloorplan().GetRoomName(room_highlighted);
  string full_name("");
  for (const auto& word : name) {
    full_name = full_name + string(" ") + word;
  }  

  //----------------------------------------------------------------------
  const int width = static_cast<int>(round(scale * kWidthRatio * viewport[2]));
  const int height = 
    width * room_thumbnails[room_highlighted].height() / room_thumbnails[room_highlighted].width();

  
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
    
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, viewport[2], 0, viewport[3]);

  glDisable(GL_DEPTH_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  {
    glDisable(GL_TEXTURE_2D);

    glBegin(GL_QUADS);
    glColor4f(color[0], color[1], color[2], alpha / 2.0);
    
    glVertex3f(render_pos[0] - kFrameMargin, viewport[3] - (render_pos[1] + kTextHeight - 3 * kFrameMargin - kTextHeight), 0.0f);
    glVertex3f(render_pos[0] - kFrameMargin, viewport[3] - (render_pos[1] + kTextHeight + height + kFrameMargin), 0.0f);
    glVertex3f(render_pos[0] + width + kFrameMargin, viewport[3] - (render_pos[1] + kTextHeight + height + kFrameMargin), 0.0f);
    glVertex3f(render_pos[0] + width + kFrameMargin, viewport[3] - (render_pos[1] + kTextHeight - 3 * kFrameMargin - kTextHeight), 0.0f);
    
    glEnd();
    glEnable(GL_TEXTURE_2D);
  }
  
  {
    //????
    //thumbnail_texid = main_widget->bindTexture(room_thumbnails[room_highlighted]);
    
    glBindTexture(GL_TEXTURE_2D, thumbnail_texid);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 room_thumbnails[room_highlighted].width(),
                 room_thumbnails[room_highlighted].height(),
                 0,
                 GL_BGRA,
                 GL_UNSIGNED_BYTE,
                 room_thumbnails[room_highlighted].bits());
    
    glBegin(GL_QUADS);
    glColor4f(1, 1, 1, alpha);
    
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(render_pos[0], viewport[3] - (render_pos[1] + kTextHeight), 0.0f);
    
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(render_pos[0], viewport[3] - (render_pos[1] + kTextHeight + height), 0.0f);
    
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(render_pos[0] + width, viewport[3] - (render_pos[1] + kTextHeight + height), 0.0f);
    
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(render_pos[0] + width, viewport[3] - (render_pos[1] + kTextHeight), 0.0f);
    
    glEnd();
  }
               
  {
    glDisable(GL_TEXTURE_2D);
    glColor4f(1, 1, 1, alpha);
    main_widget->renderText(render_pos[0], render_pos[1], full_name.c_str());
    glEnable(GL_TEXTURE_2D);
  }

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}

}  // namespace structured_indoor_modeling
