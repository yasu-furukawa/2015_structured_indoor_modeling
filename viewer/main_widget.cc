#include "main_widget.h"

#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>
#include <OpenGL/glu.h>
#include <QMouseEvent>

#include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>

using namespace Eigen;
using namespace std;

const int kNumBuffers = 2;

namespace {

bool IsInside(const LineRoom& line_room, const Vector2d& point) {
  const cv::Point2f point_tmp(point[0], point[1]);

  vector<cv::Point> contour;
  for (int w = 0; w < (int)line_room.walls.size(); ++w) {
    contour.push_back(cv::Point(line_room.walls[w][0], line_room.walls[w][1]));
  }

  if (cv::pointPolygonTest(contour, point_tmp, true) >= 0.0)
    return true;
  else
    return false;
}
  
}  // namespace

MainWidget::MainWidget(const Configuration& configuration, QWidget *parent) :
  QGLWidget(parent),
  configuration(configuration),
  navigation(panorama_renderers) {

  floorplan_renderer.Init(configuration.data_directory);
  panorama_renderers.resize(configuration.panorama_configurations.size());
  for (int p = 0; p < static_cast<int>(panorama_renderers.size()); ++p) {
    panorama_renderers[p].Init(configuration.panorama_configurations[p], this);
  }
  polygon_renderer.Init(configuration.data_directory);

  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);
  
  navigation.Init();

  SetPanoramaToRoom();
  
  current_width = current_height = -1;

  simple_click_time.start();
  double_click_time.start();
  simple_click_time_offset_by_move = 0.0;
  mouse_down = false;
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width(), height(), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);    
      
    glBindRenderbuffer(GL_RENDERBUFFER, renderids[i]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width(), height());
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
      
    glBindFramebuffer(GL_FRAMEBUFFER, frameids[i]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texids[i], 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderids[i]);

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

void MainWidget::InitializeShaders() {
  // Override system locale until shaders are compiled
  setlocale(LC_NUMERIC, "C");

  // Compile vertex shader
  if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
    close();
  
  // Compile fragment shader
  if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
    close();
  
  // Link shader pipeline
  if (!program.link())
    close();

  // Bind shader pipeline for use
  // if (!program.bind())
  // close();
  
  // Restore system locale
  setlocale(LC_ALL, "");
}

void MainWidget::initializeGL() {
  initializeGLFunctions();
  InitializeShaders();
  //qglClearColor(Qt::black);
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  for (int p = 0; p < static_cast<int>(panorama_renderers.size()); ++p)
    panorama_renderers[p].InitGL();
  
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
  const double kMinDistance = 10;
  const double kMaxDistance = 120000;
  gluPerspective(navigation.GetFieldOfViewInDegrees(),
                 width() / static_cast<double>(height()), kMinDistance, kMaxDistance);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3d center = navigation.GetCenter();
  const Vector3d direction = navigation.GetDirection();

  gluLookAt(center[0], center[1], center[2],
            center[0] + direction[0], center[1] + direction[1], center[2] + direction[2],
            0, 0, 1);


  glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
  glGetDoublev( GL_PROJECTION_MATRIX, projection );
  glGetIntegerv( GL_VIEWPORT, viewport );  
}

void MainWidget::RenderFloorplan(const double alpha) {
  FloorplanStyle style;
  style.outer_style.stroke_color = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
  style.outer_style.fill_color   = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
  style.outer_style.stroke_width = 1.0;

  style.inner_style.stroke_color = Eigen::Vector3f(0.7f, 0.7f, 0.7f);
  style.inner_style.fill_color   = Eigen::Vector3f(0.7f, 0.7f, 0.7f);
  style.inner_style.stroke_width = 1.0;

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  floorplan_renderer.Render(style, alpha);

  glDisable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}

void MainWidget::RenderPanorama(const double alpha) {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnable(GL_TEXTURE_2D);

  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    panorama_renderers[navigation.GetCameraPanorama().start_index].Render(alpha);
    break;
  }
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition: {
    panorama_renderers[navigation.GetCameraBetweenPanoramaAndAir().camera_panorama.start_index].Render(alpha);
    break;
  }
  default: {
  }
  }

  glDisable(GL_TEXTURE_2D);
}  

/*
void MainWidget::RenderQuad(const double alpha) {
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, alpha);
  glTexCoord2f(0, 0);
  glVertex3f(0.0f, 0.0f, 0.0f);
    
  glColor4f(1, 1, 1, alpha);
  glTexCoord2f(1, 0);
  glVertex3f(width(), 0.0f, 0.0f);
    
  glColor4f(1, 1, 1, alpha);
  glTexCoord2f(1, 1);
  glVertex3f(width(), height(), 0.0f);
  
  glColor4f(1, 1, 1, alpha);
  glTexCoord2f(0, 1);
  glVertex3f(0.0f, height(), 0.0f);
  glEnd();
}
*/

void MainWidget::RenderPanoramaTransition() {
  // Render the source pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);    
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[navigation.GetCameraPanorama().start_index].Render(1.0);

  // Render the target pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[1]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[navigation.GetCameraPanorama().end_index].Render(1.0);

  // Blend the two.
  const double weight_start = navigation.Progress();
  // const double weight_end = 1.0 - weight_start;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnable(GL_TEXTURE_2D);
    
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
    
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, width(), 0, height());
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //////////////////////////////////////////////////////////////////////
  if (!program.bind()) {
    cerr << "Cannot bind." << endl;
    exit (1);
  }

  program.setUniformValue("weight",
                          static_cast<float>(weight_start));
  
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texids[0]);
  glEnable(GL_TEXTURE_2D);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, texids[1]);
  glEnable(GL_TEXTURE_2D);

  glDisable(GL_DEPTH_TEST);
  
  program.setUniformValue("tex0", 0);
  program.setUniformValue("tex1", 1);

  glBegin(GL_QUADS);
  glDisable(GL_DEPTH_TEST);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);

  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(width(), 0.0f, 0.0f);

  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(width(), height(), 0.0f);

  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(0, height(), 0.0f);
  glEnd();
  
  program.release();

  glActiveTexture(GL_TEXTURE1);
  glDisable(GL_TEXTURE_2D);

  glActiveTexture(GL_TEXTURE0);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}  

void MainWidget::RenderPolygon(const double alpha,
                               const double height_adjustment) {
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);

  polygon_renderer.RenderWallAll(navigation.GetCenter(),
                                 alpha,
                                 height_adjustment,
                                 panorama_to_room[navigation.GetCameraPanorama().start_index]);
  // polygon_renderer.RenderWireframeAll(alpha);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
}

void MainWidget::paintGL() {  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  SetMatrices();

  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    if (!RightAfterSimpleClick(0.0)) {
      RenderPanorama(1.0);
      // RenderFloorplan();
    } else {
      const double alpha = Fade();
      RenderPanorama(1.0 - alpha * 0.7);
      RenderFloorplan(alpha / 2.0);
      RenderPolygon(alpha / 2.0, HeightAdjustment());
    }
    break;
  }
  case kPanoramaTransition: {
    RenderPanoramaTransition();
    break;
  }
  case kAir:
  case kAirTransition: {
    if (RightAfterSimpleClick(0.0)) {
      const double alpha = Fade();
      RenderFloorplan(alpha);
    }
    const double kNoHeightAdjustment = 0.0;
    RenderPolygon(1.0 / 3.0, kNoHeightAdjustment);
    break;
  }
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition: {
    RenderPanorama(1.0 - navigation.Progress());
    // RenderFloorplan();
    const double kNoHeightAdjustment = 0.0;
    RenderPolygon(1.0 / 3.0, kNoHeightAdjustment);
    break;
  }
  default: {
    ;
  }
  }

  //if (!RightAfterSimpleClick(kRenderMargin))
  //simple_click_time_offset_by_move = 0;
}

int MainWidget::FindPanoramaFromAirClick(const Eigen::Vector2d& pixel) const {
  int best_index = -1;
  double best_distance = 0.0;
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    GLdouble winX, winY, winZ;
    
    gluProject(panorama_renderers[p].GetCenter()[0],
               panorama_renderers[p].GetCenter()[1],
               panorama_renderers[p].GetCenter()[2],
               modelview, projection, viewport,
               &winX, &winY, &winZ);

    const double distance = (pixel - Vector2d(winX, winY)).norm();
    if (best_index == -1 || distance < best_distance) {
      best_distance = distance;
      best_index = p;
    }
  }

  return best_index;
}
  
//----------------------------------------------------------------------
// GUI
//----------------------------------------------------------------------
void MainWidget::mousePressEvent(QMouseEvent *e) {
  mousePressPosition = QVector2D(e->localPos());
  mouse_down = true;
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e) {
  mouse_down = false;

  if (QVector2D(e->localPos()) == mousePressPosition) {
    simple_click_time.start();
    simple_click_time_offset_by_move = 0;
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
    const int index = FindPanoramaFromAirClick(Vector2d(e->localPos().x(),
                                                        viewport[3] - e->localPos().y()));
    if (0 <= index)
      navigation.AirToPanorama(index);
    break;
  }
  default: {
  }
  }
}

void MainWidget::keyPressEvent(QKeyEvent* e) {
  const double kRotationDegrees = 90.0 * M_PI / 180.0;
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
      navigation.RotateSky(-kRotationDegrees);
    }
  }
  else if (e->key() == Qt::Key_Right) {
    if (navigation.GetCameraStatus() == kPanorama) {
      navigation.RotatePanorama(-kRotationDegrees);
    } else if (navigation.GetCameraStatus() == kAir) {
      navigation.RotateSky(kRotationDegrees);
    }
  } else if (e->key() == Qt::Key_A) {
    switch (navigation.GetCameraStatus()) {
    case kPanorama: {
      navigation.PanoramaToAir();
      break;
    }
    case kAir: {      
      navigation.AirToPanorama(navigation.GetCameraPanorama().start_index);
      break;
    }
    default: {
      break;
    }
    }
  }
}

void MainWidget::keyReleaseEvent(QKeyEvent *) {  
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
      navigation.RotatePanorama(Vector3d(diff.x(), diff.y(), 0.0));
      break;
    }
    case kAir: {
      diff /= 800.0;
      Vector3d direction = navigation.GetDirection();
      direction[2] = 0.0;
      Vector3d orthogonal(-direction[1], direction[0], 0.0);
      navigation.MoveAir(diff[0] * orthogonal + diff[1] * direction);
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
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition: {
    navigation.Tick();
    updateGL();
    break;
  }
  case kPanorama: {
    if (RightAfterSimpleClick(kRenderMargin)) {
      updateGL();
      break;
    }
  }
  case kAir:
    if (RightAfterSimpleClick(kRenderMargin)) {
      updateGL();
      break;
    }
  }
}

bool MainWidget::RightAfterSimpleClick(const double margin) {
  const double kDoubleClickMargin = 0.5;
  if ((simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 > kFadeInSeconds - margin &&
      (simple_click_time.elapsed() - simple_click_time_offset_by_move) / 1000.0 < kFadeOutSeconds + margin &&
      (double_click_time.elapsed() - (simple_click_time.elapsed() - simple_click_time_offset_by_move)) / 1000.0 > kDoubleClickMargin - margin) {
    return true;
  }
  else {
    return false;
  }
}

double MainWidget::Progress() {
  return ProgressFunction(simple_click_time.elapsed() / 1000.0,
                          simple_click_time_offset_by_move / 1000.0);
}

double MainWidget::Fade() {
  return FadeFunction(simple_click_time.elapsed() / 1000.0,
                      simple_click_time_offset_by_move / 1000.0);
}

double MainWidget::HeightAdjustment() {
  return HeightAdjustmentFunction(simple_click_time.elapsed() / 1000.0,
                                  simple_click_time_offset_by_move / 1000.0);
}

double MainWidget::ProgressFunction(const double elapsed, const double offset) {
  //if (offset < kFadeInSeconds)
  //return ((elapsed) - kFadeInSeconds) / (kFadeOutSeconds - kFadeInSeconds);
  //else
  return max(0.0,
             ((elapsed - offset) - kFadeInSeconds) /
             (kFadeOutSeconds - kFadeInSeconds));
}

double MainWidget::FadeFunction(const double elapsed, const double offset) {
  // When a mouse keeps moving (offset is large, we draw with a full opacity).
  const double progress = ProgressFunction(elapsed, offset);
  if (progress < 0.1) {
    if (offset > kFadeInSeconds)
      return 1.0;
    else
      return 10.0 * progress;
  } else if (progress > 0.75) {
    return 1.0 - (progress - 0.75) * 4.0;
  } else {
    return 1.0;
  }
}

double MainWidget::HeightAdjustmentFunction(const double elapsed,
                                            const double offset) {
  // When a mouse keeps moving (offset is large, we draw with a full opacity).
  if (offset > kFadeInSeconds)
    return 1.0;

  const double progress = ProgressFunction(elapsed, offset);
  return min(1.0, 8.0 * progress);
}

void MainWidget::SetPanoramaToRoom() {
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    const LineFloorplan& line_floorplan = polygon_renderer.GetLineFloorplan();
    const Eigen::Matrix3d& floorplan_to_global = polygon_renderer.GetFloorplanToGlobal();

    const Vector3d global_center = panorama_renderers[p].GetCenter();
    const Vector3d floorplan_center = floorplan_to_global.transpose() * global_center;
    const Vector2d floorplan_center2(floorplan_center[0], floorplan_center[1]);
           
    int room_id = -1;
    for (int room = 0; room < (int)line_floorplan.line_rooms.size(); ++room) {
      if (IsInside(line_floorplan.line_rooms[room], floorplan_center2)) {
        room_id = room;
        break;
      }
    }
    panorama_to_room[p] = room_id;
  }
}
