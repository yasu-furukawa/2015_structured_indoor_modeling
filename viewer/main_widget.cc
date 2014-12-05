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
  navigation(panoramas) {

  panoramas.resize(configuration.panorama_configurations.size());
  for (int p = 0; p < static_cast<int>(panoramas.size()); ++p) {
    panoramas[p].Init(configuration.panorama_configurations[p], this);
  }

  polygon_renderer.Init(configuration.data_directory);

  setFocusPolicy(Qt::ClickFocus);
  
  navigation.Init();

  current_width = current_height = -1;

  init_shader = false;
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

  for (int p = 0; p < static_cast<int>(panoramas.size()); ++p)
    panoramas[p].InitGL();
  
  // Use QBasicTimer because its faster than QTimer
  timer.start(1000 / 60, this);
}

void MainWidget::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);

  if (w != current_width || h != current_height) {
    FreeResources();
    AllocateResources();
    init_shader = true;
  }  
}

void MainWidget::SetMatrices() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  const double kMinDistance = 10;
  const double kMaxDistance = 40000;
  const double kFieldOfView = 100.0;
  gluPerspective(kFieldOfView, width() / static_cast<double>(height()), kMinDistance, kMaxDistance);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3d center = navigation.GetCenter();
  const Vector3d direction = navigation.GetDirection();
  gluLookAt(center[0], center[1], center[2],
            center[0] + direction[0], center[1] + direction[1], center[2] + direction[2],
            0, 0, 1);
}

void MainWidget::RenderPanorama() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnable(GL_TEXTURE_2D);
  panoramas[navigation.GetCameraOnGround().start_index].Render();
}  

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

void MainWidget::RenderPanoramaTransition() {
  // Render the source pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);    
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panoramas[navigation.GetCameraOnGround().start_index].Render();

  // Render the target pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[1]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panoramas[navigation.GetCameraOnGround().end_index].Render();

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

void MainWidget::RenderRooms() {
  //polygon_renderer.RenderWallAll();
  polygon_renderer.RenderWireframeAll();
}

void MainWidget::paintGL() {  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  SetMatrices();

  switch (navigation.GetCameraStatus()) {
  case kPanoramaStop:
    RenderPanorama();
    break;
  case kPanoramaTransition:
    RenderPanoramaTransition();
    break;
  }

  RenderRooms();
}

//----------------------------------------------------------------------
// GUI
//----------------------------------------------------------------------
void MainWidget::mousePressEvent(QMouseEvent *e) {
  mousePressPosition = QVector2D(e->localPos());
}

void MainWidget::mouseReleaseEvent(QMouseEvent *) {
}

void MainWidget::keyPressEvent(QKeyEvent* e) {
  const double kRotationDegrees = 90.0 * M_PI / 180.0;
  if (e->key() == Qt::Key_Up) {
    if (navigation.GetCameraStatus() == kPanoramaStop) {
      navigation.MoveForwardOnGround();
    }
  } else if (e->key() == Qt::Key_Down) {
    if (navigation.GetCameraStatus() == kPanoramaStop) {
      navigation.MoveBackwardOnGround();
    }
  } else if (e->key() == Qt::Key_Left) {
    if (navigation.GetCameraStatus() == kPanoramaStop) {
      
      navigation.RotateOnGround(kRotationDegrees);
    }
  } else if (e->key() == Qt::Key_Right) {
    if (navigation.GetCameraStatus() == kPanoramaStop) {
      navigation.RotateOnGround(-kRotationDegrees);
    }
  }
}

void MainWidget::keyReleaseEvent(QKeyEvent *) {
}

void MainWidget::mouseMoveEvent(QMouseEvent *e) {
  QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;
  mousePressPosition = QVector2D(e->localPos());
  diff /= 400.0;
  navigation.RotateOnGround(Vector3d(diff.x(), -diff.y(), 0.0));      
  
  updateGL();
}

void MainWidget::timerEvent(QTimerEvent *) {
  if (navigation.GetCameraStatus() == kPanoramaStop)
    return;

  navigation.Tick();
  updateGL();
}
