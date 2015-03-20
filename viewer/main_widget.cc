#include "main_widget.h"
#include "../base/panorama.h"

#include <fstream>
#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>
#include <QMouseEvent>

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

#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

#include "main_widget_util.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int kNumBuffers = 2;

const double MainWidget::kRenderMargin = 0.2;
const double MainWidget::kFadeInSeconds = 0.2;
const double MainWidget::kFadeOutSeconds = 1.5;
// The background color is not exactly 0, because we want to treat a
// pure black pixel in panorama images as holes. Fragment shader
// handles the pure black pixel in a special way, and we want the
// background color to be intensity 1 (avoid pure black).
const Eigen::Vector3f MainWidget::kBackgroundColor = Eigen::Vector3f(0.005, 0.005, 0.005);

MainWidget::MainWidget(const Configuration& configuration, QWidget *parent) :
  QGLWidget(parent),
  file_io(configuration.data_directory),
  floorplan(file_io.GetFloorplanFinal()),
  indoor_polygon(file_io.GetIndoorPolygonFinal()),
  object_renderer(floorplan, indoor_polygon, file_io.GetObjectDetectionsFinal()),
  polygon_renderer(floorplan),
  indoor_polygon_renderer(indoor_polygon),
  floorplan_renderer(floorplan, indoor_polygon),
  panel_renderer(floorplan, viewport),  
  navigation(configuration,
             floorplan,
             panoramas,
             panorama_to_room,
             room_to_panorama) {

  // Renderer initialization.
  {
    InitPanoramasPanoramaRenderers();
    object_renderer.Init(configuration.data_directory);
    polygon_renderer.Init(configuration.data_directory, this);
    indoor_polygon_renderer.Init(configuration.data_directory, this);
    floorplan_renderer.Init();
    panel_renderer.Init(configuration.data_directory);
  }

  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);
  
  navigation.Init();

  SetPanoramaToRoom(floorplan, panorama_renderers, &panorama_to_room);
  SetRoomToPanorama(floorplan, panorama_renderers, &room_to_panorama);
  SetPanoramaDistanceTable(panorama_renderers, &panorama_distance_table);
  
  current_width = current_height = -1;

  prev_animation_linear = 0.0;
  prev_animation_trapezoid = 0.0;
  
  fresh_screen_for_panorama = true;
  fresh_screen_for_air = true;
  fresh_screen_for_floorplan = true;

  simple_click_time.start();
  double_click_time.start();
  object_animation_time.start();
  simple_click_time_offset_by_move = 0.0;
  mouse_down = false;

  polygon_or_indoor_polygon = false;
}

MainWidget::~MainWidget() {
  FreeResources();
}

void MainWidget::AllocateResources() {
  glGenTextures(kNumBuffers, texids);
  glGenFramebuffers(kNumBuffers, frameids);
  glGenRenderbuffers(kNumBuffers, renderids);
    
  for (int i = 0; i < kNumBuffers; ++i) {
    glBindTexture(GL_TEXTURE_2D, texids[i]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width(), height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);    
      
    glBindRenderbuffer(GL_RENDERBUFFER, renderids[i]);
    // glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width(), height());
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width(), height());
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
      
    glBindFramebuffer(GL_FRAMEBUFFER, frameids[i]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texids[i], 0);
    // glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderids[i]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderids[i]);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER); 
    if (status != GL_FRAMEBUFFER_COMPLETE) {
      cerr << "not complete" << endl;
      exit (1);
    }
  }
}

void MainWidget::FreeResources() {
  if (current_width != -1) {
    glDeleteTextures(kNumBuffers, texids);
    glDeleteFramebuffers(kNumBuffers, frameids);
    glDeleteRenderbuffers(kNumBuffers, renderids);
  }
}

void MainWidget::InitPanoramasPanoramaRenderers() {
  const int kMaxPanoramaId = 100;
  vector<int> panorama_ids;
  for (int p = 0; p < kMaxPanoramaId; ++p) {
    ifstream ifstr;
    ifstr.open(file_io.GetPanoramaImage(p).c_str());
    if (!ifstr.is_open())
      continue;
    panorama_ids.push_back(p);
  }
  if (panorama_ids.empty()) {
    cerr << "No panorama." << endl;
    exit (1);
  }

  panoramas.resize(panorama_ids.size());
  panorama_renderers.resize(panorama_ids.size());
  for (int i = 0; i < (int)panorama_ids.size(); ++i) {
    panoramas[i].Init(file_io, panorama_ids[i]);
    panorama_renderers[i].Init(file_io, panorama_ids[i], &panoramas[i], this);
  }
}
  
void MainWidget::InitializeShaders() {
  // Override system locale until shaders are compiled
  setlocale(LC_NUMERIC, "C");

  // Compile vertex shader
  if (!blend_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/blend_vshader.glsl"))
    close();
  
  // Compile fragment shader
  if (!blend_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/blend_fshader.glsl"))
    close();
  
  // Link shader pipeline
  if (!blend_program.link())
    close();

  // Bind shader pipeline for use
  // if (!blend_program.bind())
  // close();

  // Compile vertex shader
  if (!panorama_program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/panorama_vshader.glsl"))
    close();
  // Compile fragment shader
  if (!panorama_program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/panorama_fshader.glsl"))
    close();
  // Link shader pipeline
  if (!panorama_program.link())
    close();

  // Restore system locale
  setlocale(LC_ALL, "");
}

void MainWidget::initializeGL() {
  initializeOpenGLFunctions();
  InitializeShaders();
  glClearColor(kBackgroundColor[0], kBackgroundColor[1], kBackgroundColor[2], 0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  object_renderer.InitGL();
  polygon_renderer.InitGL();
  indoor_polygon_renderer.InitGL();
  floorplan_renderer.InitGL(this);
  for (int p = 0; p < static_cast<int>(panorama_renderers.size()); ++p)
    panorama_renderers[p].InitGL();
  panel_renderer.InitGL(this);
  
  // Use QBasicTimer because its faster than QTimer
  timer.start(1000 / 60, this);
}

void MainWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  if (w != current_width || h != current_height) {
    FreeResources();
    AllocateResources();
  }  
}

void MainWidget::SetMatrices() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const double max_distance = navigation.GetFloorplanHeight() * 2.0;
  const double min_distance = max_distance / 10000.0;
  gluPerspective(navigation.GetFieldOfViewInDegrees(),
                 width() / static_cast<double>(height()), min_distance, max_distance);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3d center = navigation.GetCenter();
  const Vector3d direction = navigation.GetDirection();
  if (direction.norm() == 0.0) {
    cerr << "ZERO" << endl;
    exit (1);
  }

  if (std::isnan(center[0]) || std::isnan(center[1]) || std::isnan(center[2]) ||
      std::isnan(direction[0]) || std::isnan(direction[1]) || std::isnan(direction[2]))
    cerr << "black?" << endl
         << center << endl
         << direction << endl;

  gluLookAt(center[0], center[1], center[2],
            center[0] + direction[0], center[1] + direction[1], center[2] + direction[2],
            0, 0, 1);

  GLint viewport_old[4];
  GLdouble modelview_old[16];
  GLdouble projection_old[16];
  for (int i = 0; i < 4; ++i)
    viewport_old[i] = viewport[i];
  for (int i = 0; i < 16; ++i) {
    modelview_old[i] = modelview[i];
    projection_old[i] = projection[i];
  }

  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );

  for (int i = 0; i < 4; ++i) {
    if (viewport_old[i] != viewport[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (modelview_old[i] != modelview[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (projection_old[i] != projection[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      fresh_screen_for_floorplan = true;
      break;
    }
  }

  if (prev_animation_linear != AnimationLinear()) {
    fresh_screen_for_panorama = true;
    fresh_screen_for_air = true;
    fresh_screen_for_floorplan = true;
    prev_animation_linear = AnimationLinear();  
  }

  if (prev_animation_trapezoid != AnimationTrapezoid()) {
    fresh_screen_for_panorama = true;
    fresh_screen_for_air = true;
    fresh_screen_for_floorplan = true;
    prev_animation_trapezoid = AnimationTrapezoid();  
  }
}

void MainWidget::PaintPanorama() {
  const bool kDepthOrderHeightAdjustment = true;
  // When mouse is moving, this is called in every frame, and
  // becomes slow. So, we do this only when the mouse is not
  // moving. Interaction
  if (fresh_screen_for_panorama && !mouse_down) {
    RenderPolygonLabels(panorama_to_room[navigation.GetCameraPanorama().start_index],
                        AnimationLinear(),
                        kDepthOrderHeightAdjustment);
    fresh_screen_for_panorama = false;
  }
  
  if (RightAfterSimpleClick(0.0)) {
    const double trapezoid = AnimationTrapezoid();
    RenderPanorama(1.0 - trapezoid * 0.7);
    // Checks if any room should be highlighted.
    int room_highlighted = -1;
    if (!mouse_down)
      room_highlighted = FindRoomHighlighted(Vector2i(mouseMovePosition[0],
                                                      mouseMovePosition[1]));
    RenderPolygon(panorama_to_room[navigation.GetCameraPanorama().start_index],
                  trapezoid / 2.0,
                  AnimationLinear(),
                  kDepthOrderHeightAdjustment,
                  room_highlighted);
    RenderThumbnail(1.0, room_highlighted, this);
  } else {
    RenderPanorama(1.0);
  }    
}

void MainWidget::PaintAir() {
  const bool kUniformHeightAdjustment = false;
  const double kNoHeightAdjustment = 0.0;
  if (fresh_screen_for_air && !mouse_down) {
    RenderPolygonLabels(-1, kNoHeightAdjustment, kUniformHeightAdjustment);
    fresh_screen_for_air = false;
  }
  
  if (RightAfterSimpleClick(0.0)) {
    const double trapezoid = AnimationTrapezoid();
    RenderTexturedPolygon(1.0 - trapezoid * 0.7);
    RenderObjects(1.0 - trapezoid * 0.7);
    
    int room_highlighted = -1;
    if (!mouse_down)
      room_highlighted = FindRoomHighlighted(Vector2i(mouseMovePosition[0],
                                                      mouseMovePosition[1]));
    
    RenderPolygon(-1, trapezoid / 2.0, 1.0 - trapezoid, kUniformHeightAdjustment, room_highlighted);
    RenderThumbnail(1.0, room_highlighted, this);
  } else {
    RenderTexturedPolygon(1.0);
    RenderObjects(1.0);
  }    
}

void MainWidget::PaintFloorplan() {
  if (fresh_screen_for_floorplan && !mouse_down) {
    RenderFloorplanLabels();
    fresh_screen_for_floorplan = false;
  }
  
  if (RightAfterSimpleClick(0.0)) {
    const bool kEmphasize = true;
    RenderFloorplan(1.0, kEmphasize, AnimationTrapezoid());    
    RenderAllRoomNames(1.0, this);
    
    int room_highlighted = -1;
    if (!mouse_down)
      room_highlighted = FindRoomHighlighted(Vector2i(mouseMovePosition[0],
                                                      mouseMovePosition[1]));
    if (room_highlighted != -1)
      RenderThumbnail(1.0, room_highlighted, this);
  } else {
    const bool kNoEmphasize = false;
    RenderFloorplan(1.0, kNoEmphasize, 0);
    RenderAllRoomNames(1.0, this);
  }    
}
  
void MainWidget::paintGL() {
  ClearDisplay();
    
  SetMatrices();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  const bool kFlip = true;
  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    PaintPanorama();
    break;
  }
  case kPanoramaTransition: {
    RenderPanoramaTransition(navigation.GetCameraPanorama().start_index,
                             navigation.GetCameraPanorama().end_index,
                             navigation.ProgressInverse());
    break;
  }
  case kAir: {
    PaintAir();
    break;
  }
  case kAirTransition: {
    RenderTexturedPolygon(1.0);
    RenderObjects(1.0);
    break;
  }
  case kFloorplan: {
    PaintFloorplan();
    break;
  }
  case kFloorplanTransition: {
    RenderFloorplan(1.0, false, 0);
    RenderAllRoomNames(1.0, this);
    break;
  }
  case kPanoramaToAirTransition: {
    RenderPanoramaToAirTransition();
    break;
  }
  case kAirToPanoramaTransition: {
    RenderPanoramaToAirTransition(kFlip);
    break;
  }
  case kPanoramaToFloorplanTransition: {
    RenderPanoramaToFloorplanTransition();
    break;
  }
  case kFloorplanToPanoramaTransition: {
    RenderPanoramaToFloorplanTransition(kFlip);
    break;
  }
  case kAirToFloorplanTransition: {
    RenderAirToFloorplanTransition();
    break;
  }
  case kFloorplanToAirTransition: {
    RenderAirToFloorplanTransition(kFlip);
    break;
  }
  case kPanoramaTour: {
    RenderPanoramaTour();
    break;
  }
  default: {
    ;
  }
  }

  //if (!RightAfterSimpleClick(kRenderMargin))
  //simple_click_time_offset_by_move = 0;
}

  
//----------------------------------------------------------------------
// GUI
//----------------------------------------------------------------------
void MainWidget::mousePressEvent(QMouseEvent *e) {
  mousePressPosition = QVector2D(e->localPos());
  mouse_down = true;

  if (e->button() == Qt::RightButton) {
    switch (navigation.GetCameraStatus()) {
    case kPanorama: {
      navigation.PanoramaToAir();
      break;
    }
    case kAir: {      
      navigation.AirToPanorama(navigation.GetCameraPanorama().start_index);
      break;
    }
    case kFloorplan: {      
      navigation.FloorplanToPanorama(navigation.GetCameraPanorama().start_index);
      break;
    }
    default: {
    }
    }
  }
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e) {
  mouse_down = false;

  if (QVector2D(e->localPos()) == mousePressPosition) {
    // This may trigger a tour.
    if (navigation.GetCameraStatus() == kPanorama &&
        RightAfterSimpleClick(0.0)) {
      const int room_highlighted = FindRoomHighlighted(Vector2i(mousePressPosition[0],
                                                                mousePressPosition[1]));
      //simple_click_time.start();
      if (room_highlighted != -1) {
        vector<int> indexes;

        FindPanoramaPath(panorama_renderers,
                         panorama_distance_table,
                         navigation.GetCameraPanorama().start_index,
                         room_to_panorama[room_highlighted],
                         &indexes);
        navigation.TourToPanorama(indexes);
        // Not perfect, the following line is good enough.
        simple_click_time_offset_by_move = 0;
      }
    } else if (navigation.GetCameraStatus() == kAir &&
               RightAfterSimpleClick(0.0)) {
      const int room_highlighted = FindRoomHighlighted(Vector2i(mousePressPosition[0],
                                                                mousePressPosition[1]));
      if (room_highlighted != -1) {
        navigation.AirToPanorama(room_to_panorama[room_highlighted]);
        // Not perfect, the following line is good enough.
        simple_click_time_offset_by_move = 0;
      }
    } else if (navigation.GetCameraStatus() == kFloorplan &&
               RightAfterSimpleClick(0.0)) {
      const int room_highlighted = FindRoomHighlighted(Vector2i(mousePressPosition[0],
                                                                mousePressPosition[1]));
      if (room_highlighted != -1) {
        navigation.FloorplanToPanorama(room_to_panorama[room_highlighted]);
        // Not perfect, the following line is good enough.
        simple_click_time_offset_by_move = 0;
      }      
    } else {
      simple_click_time.start();
      simple_click_time_offset_by_move = 0;
    }
  }
}

void MainWidget::mouseDoubleClickEvent(QMouseEvent *e) {
  double_click_time.start();
  
  mouse_down = true;
  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    Vector3d direction = navigation.GetDirection();
    direction[2] = 0.0;
    direction.normalize();
    Vector3d up(0, 0, 1);
    Vector3d x_axis = direction.cross(up);
    
    const double x_position = 2.0 * (e->localPos().x() / (double)viewport[2] - 0.5);
    
    Vector3d new_direction =
      direction + x_axis * x_position * tan(navigation.GetFieldOfViewInDegrees() / 2.0 * M_PI / 180.0);
    navigation.MovePanorama(new_direction);
    break;
  }
  case kAir: {
    const int index = FindPanoramaFromAirFloorplanClick(panorama_renderers,
                                                        Vector2d(e->localPos().x(),
                                                                 viewport[3] - e->localPos().y()),
                                                        viewport,
                                                        modelview,
                                                        projection);
    if (0 <= index)
      navigation.AirToPanorama(index);
    break;
  }
  case kFloorplan: {
    const int index = FindPanoramaFromAirFloorplanClick(panorama_renderers,
                                                        Vector2d(e->localPos().x(),
                                                                 viewport[3] - e->localPos().y()),
                                                        viewport,
                                                        modelview,
                                                        projection);
    if (0 <= index)
      navigation.FloorplanToPanorama(index);
    break;
  }
  default: {
  }
  }
}

void MainWidget::keyPressEvent(QKeyEvent* e) {
  const double kRotationDegrees = 45.0 * M_PI / 180.0;
  if (e->key() == Qt::Key_Up) {
    if (navigation.GetCameraStatus() == kPanorama) {
      navigation.MoveForwardPanorama();
    }
  } else if (e->key() == Qt::Key_Down) {
    if (navigation.GetCameraStatus() == kPanorama) {
      navigation.MoveBackwardPanorama();
    }
  } else if (e->key() == Qt::Key_Left) {
    if (navigation.GetCameraStatus() == kPanorama) {
      navigation.RotatePanorama(kRotationDegrees);
    } else if (navigation.GetCameraStatus() == kAir) {
      navigation.RotateAir(-kRotationDegrees);
    } else if (navigation.GetCameraStatus() == kFloorplan) {
      navigation.RotateFloorplan(-kRotationDegrees);
    }
  }
  else if (e->key() == Qt::Key_Right) {
    if (navigation.GetCameraStatus() == kPanorama) {
      navigation.RotatePanorama(-kRotationDegrees);
    } else if (navigation.GetCameraStatus() == kAir) {
      navigation.RotateAir(kRotationDegrees);
    } else if (navigation.GetCameraStatus() == kFloorplan) {
      navigation.RotateFloorplan(kRotationDegrees);
    }
  } else if (e->key() == Qt::Key_A) {
    if (navigation.GetCameraStatus() == kAir)
      navigation.AirToPanorama(navigation.GetCameraPanorama().start_index);
    else if (navigation.GetCameraStatus() == kFloorplan)
      navigation.FloorplanToPanorama(navigation.GetCameraPanorama().start_index);
  } else if (e->key() == Qt::Key_S) {
    if (navigation.GetCameraStatus() == kPanorama)
      navigation.PanoramaToAir();
    else if (navigation.GetCameraStatus() == kFloorplan)
      navigation.FloorplanToAir();
  } else if (e->key() == Qt::Key_D) {
    if (navigation.GetCameraStatus() == kPanorama)
      navigation.PanoramaToFloorplan();
    else if (navigation.GetCameraStatus() == kAir)
      navigation.AirToFloorplan();
  } else if (e->key() == Qt::Key_O) {
    object_renderer.Toggle();
    updateGL();
  } else if (e->key() == Qt::Key_P) {
    polygon_or_indoor_polygon = !polygon_or_indoor_polygon;
    updateGL();
  }  
}

void MainWidget::keyReleaseEvent(QKeyEvent *) {  
}

void MainWidget::wheelEvent(QWheelEvent* e) {
  switch (navigation.GetCameraStatus()) {
  case kAir:
  case kFloorplan:
    {
    if (e->orientation() == Qt::Vertical) {
      navigation.ScaleAirFloorplanFieldOfView(e->delta());
      updateGL();
    }
    break;
  }
  default: {
    break;
  }
  }
}

void MainWidget::mouseMoveEvent(QMouseEvent *e) {
  QVector2D diff = QVector2D(e->localPos()) - mouseMovePosition;
  mouseMovePosition = QVector2D(e->localPos());

  if (RightAfterSimpleClick(0.0)) {
    simple_click_time_offset_by_move =
      max(0, -1 + simple_click_time.elapsed() - static_cast<int>(round(1000 * kFadeInSeconds)));
  }
  
  if (mouse_down) {
    switch (navigation.GetCameraStatus()) {
    case kPanorama: {
      diff /= 400.0;
      navigation.RotatePanorama(diff.x(), diff.y());
      break;
    }
    case kAir:
    case kFloorplan: {
      diff /= 100.0;
      Vector3d direction = navigation.GetDirection();
      direction[2] = 0.0;
      direction.normalize();
      direction *= navigation.GetAverageDistance();
      Vector3d orthogonal(-direction[1], direction[0], 0.0);
      navigation.MoveAir(diff[0] * orthogonal + diff[1] * direction);
      navigation.MoveFloorplan(diff[0] * orthogonal + diff[1] * direction);
      break;
    }
    default: {
    }
    }
    updateGL();
  }
}

void MainWidget::timerEvent(QTimerEvent *) {
  switch (navigation.GetCameraStatus()) {
  case kPanoramaTransition:
  case kAirTransition:
  case kFloorplanTransition:
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition:
  case kPanoramaToFloorplanTransition:
  case kFloorplanToPanoramaTransition:
  case kAirToFloorplanTransition:
  case kFloorplanToAirTransition:
  case kPanoramaTour: {
    navigation.Tick();
    updateGL();
    break;
  }
  case kPanorama: {
    if (RightAfterSimpleClick(kRenderMargin)) {
      updateGL();
    }
    break;
  }
  case kAir: {
    if (RightAfterSimpleClick(kRenderMargin) || ObjectAnimation()) {
      updateGL();
    }
    break;
  }
  case kFloorplan: {
    if (RightAfterSimpleClick(kRenderMargin)) {
      updateGL();
    }
    break;
  }
  }
}  

bool MainWidget::RightAfterSimpleClick(const double margin) const {
  const double kDoubleClickMargin = 0.5;
  if ((simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 >
      kFadeInSeconds - margin &&
      (simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 <
      kFadeOutSeconds + margin &&
      (double_click_time.elapsed() -
       (simple_click_time.elapsed() - simple_click_time_offset_by_move)) / 1000.0 >
      kDoubleClickMargin - margin) {
    return true;
  }
  else {
    return false;
  }
}

  /*
double MainWidget::Progress() {
  return ProgressFunction(simple_click_time.elapsed() / 1000.0,
                          simple_click_time_offset_by_move / 1000.0,
                          kFadeInSeconds,
                          kFadeOutSeconds);
}
  */

double MainWidget::AnimationTrapezoid() {
  return AnimationTrapezoidUtil(simple_click_time.elapsed() / 1000.0,
                                simple_click_time_offset_by_move / 1000.0,
                                kFadeInSeconds,
                                kFadeOutSeconds);
}

double MainWidget::AnimationLinear() {
  return AnimationLinearUtil(simple_click_time.elapsed() / 1000.0,
                             simple_click_time_offset_by_move / 1000.0,
                             kFadeInSeconds,
                             kFadeOutSeconds);
}

double MainWidget::ObjectAnimationPosition() const {
  if (!ObjectAnimation())
    return 0.0;

  const int kInterval = 6000;  
  const int kDuration = 1000;
  const int msec = object_animation_time.elapsed();
  return (msec % kInterval) / (double)kDuration;
}
  
bool MainWidget::ObjectAnimation() const {
  // Once in 6 seconds.
  const int kInterval = 6000;
  const int kDuration = 1000;
  const int msec = object_animation_time.elapsed();
  if (msec % kInterval < kDuration)
    return true;
  else
    return false;
}
  
}  // namespace structured_indoor_modeling
