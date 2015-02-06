#include <iostream>

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

#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "panel_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const double PanelRenderer::kWidthRatio = 0.2;
const int PanelRenderer::kTextHeight = 6;
const int PanelRenderer::kFrameMargin = 5;

PanelRenderer::PanelRenderer(const Floorplan& floorplan,
                             const GLint* viewport) :
  floorplan(floorplan), viewport(viewport) {
  thumbnail_texid = 0;
}

PanelRenderer::~PanelRenderer() {
  glDeleteTextures(1, &thumbnail_texid);
}

void PanelRenderer::Init(const std::string& data_directory) {
  FileIO file_io(data_directory);

  const int room_num = floorplan.GetNumRooms();
  room_thumbnails.resize(room_num);
  for (int room = 0; room < room_num; ++room) {
    room_thumbnails[room].load(file_io.GetRoomThumbnail(room).c_str());
  }
}

void PanelRenderer::InitGL(MainWidget* /*main_widget*/) {
  initializeGLFunctions();
  glGenTextures(1, &thumbnail_texid);
}

void PanelRenderer::RenderThumbnail(const double alpha,
                                    const int room_highlighted,
                                    const Vector2i& render_pos,
                                    const Vector3d& color,
                                    const double scale,
                                    QGLWidget* qgl_widget) {
  if (room_highlighted == -1) {
    return;
  }
  const vector<string>& name = floorplan.GetRoomName(room_highlighted);
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
    qgl_widget->renderText(render_pos[0], render_pos[1], full_name.c_str());
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
