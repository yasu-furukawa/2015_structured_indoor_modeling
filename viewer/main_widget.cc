#include "main_widget.h"

#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>
#include <OpenGL/glu.h>
#include <QMouseEvent>

using namespace Eigen;
using namespace std;

const int kNumBuffers = 2;

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
  current_width = current_height = -1;

  qtime.start();
  mouse_down = false;
  double_clicked = false;
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

void MainWidget::RenderFloorplan() {
  FloorplanStyle style;
  style.outer_style.stroke_color = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  style.outer_style.fill_color   = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  style.outer_style.stroke_width = 1.0;

  style.inner_style.stroke_color = Eigen::Vector3f(0.0f, 1.0f, 0.5f);
  style.inner_style.fill_color   = Eigen::Vector3f(0.0f, 1.0f, 0.5f);
  style.inner_style.stroke_width = 1.0;

  glClear(GL_DEPTH_BUFFER_BIT);
  floorplan_renderer.Render(style);
}

void MainWidget::RenderPanorama() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[navigation.GetCameraPanorama().start_index].Render();
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
  panorama_renderers[navigation.GetCameraPanorama().start_index].Render();

  // Render the target pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[1]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[navigation.GetCameraPanorama().end_index].Render();

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

void MainWidget::RenderPolygon() {
  //polygon_renderer.RenderWallAll();
  polygon_renderer.RenderWireframeAll();
}

void MainWidget::paintGL() {  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  SetMatrices();

  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    RenderPanorama();
    // RenderFloorplan();
    if (qtime.elapsed() / 1000 < kPanoramaFadeOutSeconds) {
      RenderPolygon();
    }
    break;
  }
  case kPanoramaTransition: {
    RenderPanoramaTransition();
    // RenderFloorplan();
    // RenderPolygon();
    break;
  }
  case kAir:
  case kAirTransition: {
    if (qtime.elapsed() / 1000 < kAirFadeOutSeconds) {
      RenderFloorplan();
    }
    RenderPolygon();
    break;
  }
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition: {
    RenderPanorama();
    // RenderFloorplan();
    RenderPolygon();
    break;
  }
  default: {
    ;
  }
  }
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

  if (!double_clicked && QVector2D(e->localPos()) == mousePressPosition) {
    qtime.start();
  }

  double_clicked = false;
}

void MainWidget::mouseDoubleClickEvent(QMouseEvent *e) {
  double_clicked = true;
  cout << qtime.elapsed() << ' ';
  qtime = qtime.addSecs(1.0);
  cout << qtime.elapsed() << endl;
  
  mouse_down = true;
  switch (navigation.GetCameraStatus()) {
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

  if (mouse_down) {
    switch (navigation.GetCameraStatus()) {
    case kPanorama: {
      diff /= 400.0;
      navigation.RotatePanorama(Vector3d(diff.x(), diff.y(), 0.0));
      break;
    }
    case kAir: {
      diff /= 400.0;
      Vector3d direction = navigation.GetDirection();
      direction[2] = 0.0;
      Vector3d orthogonal(-direction[1], direction[0], 0.0);
      navigation.MoveAir(diff[0] * orthogonal + diff[1] * direction);    
      break;
    }
    default: {
      return;
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
    if (qtime.elapsed() / 1000 < kPanoramaFadeOutSeconds + 0.5) {
      updateGL();
      break;
      //}
  }
  case kAir:
    if (qtime.elapsed() / 1000 < kAirFadeOutSeconds + 0.5) {
      updateGL();
      break;
      //}
    break;
  }
  }
  }
}
