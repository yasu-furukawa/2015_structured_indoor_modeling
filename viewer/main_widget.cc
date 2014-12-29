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

const double MainWidget::kRenderMargin = 0.2;
const double MainWidget::kFadeInSeconds = 0.2;
const double MainWidget::kFadeOutSeconds = 1.5;

namespace {

bool IsInside(const Floorplan floorplan, const int room, const Vector2d& point) {
  const cv::Point2f point_tmp(point[0], point[1]);

  vector<cv::Point> contour;
  for (int w = 0; w < floorplan.GetNumWalls(room); ++w) {
    const Eigen::Vector2d local = floorplan.GetRoomVertexLocal(room, w);
    contour.push_back(cv::Point(local[0], local[1]));
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
  panel_renderer(polygon_renderer, viewport),  
  navigation(panorama_renderers, polygon_renderer, panorama_to_room, room_to_panorama) {

  panorama_renderers.resize(configuration.panorama_configurations.size());
  for (int p = 0; p < static_cast<int>(panorama_renderers.size()); ++p) {
    panorama_renderers[p].Init(configuration.panorama_configurations[p], this);
  }
  polygon_renderer.Init(configuration.data_directory, this);
  floorplan_renderer.Init(configuration.data_directory,
                          polygon_renderer.GetFloorplan().GetFloorplanToGlobal());
  panel_renderer.Init(configuration.data_directory);

  setFocusPolicy(Qt::ClickFocus);
  setMouseTracking(true);
  
  navigation.Init();

  SetPanoramaToRoom();
  SetRoomToPanorama();
  SetPanoramaDistanceTable();

  
  current_width = current_height = -1;

  prev_height_adjustment = 0.0;
  fresh_screen_for_panorama = true;
  fresh_screen_for_air = true;

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

  polygon_renderer.InitGL();
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
  const double kMinDistance = 10;
  const double kMaxDistance = 120000;
  gluPerspective(navigation.GetFieldOfViewInDegrees(),
                 width() / static_cast<double>(height()), kMinDistance, kMaxDistance);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const Vector3d center = navigation.GetCenter();
  const Vector3d direction = navigation.GetDirection();
  if (direction.norm() == 0.0) {
    cerr << "ZERO" << endl;
    exit (1);
  }

  if (isnan(center[0]) || isnan(center[1]) || isnan(center[2]) ||
      isnan(direction[0]) || isnan(direction[1]) || isnan(direction[2]))
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
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (modelview_old[i] != modelview[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      break;
    }
  }
  for (int i = 0; i < 16; ++i) {
    if (projection_old[i] != projection[i]) {
      fresh_screen_for_panorama = true;
      fresh_screen_for_air = true;
      break;
    }
  }

  if (prev_height_adjustment != HeightAdjustment()) {
    fresh_screen_for_panorama = true;
    fresh_screen_for_air = true;
  }
  prev_height_adjustment = HeightAdjustment();  
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

void MainWidget::RenderPanoramaTour() {
  int index_pair[2];
  int panorama_index_pair[2];
  double weight_pair[2];
  navigation.GetCameraPanoramaTour().GetIndexWeightPairs(1.0 - navigation.ProgressInverse(),
                                                         index_pair,
                                                         panorama_index_pair,
                                                         weight_pair);
  RenderPanoramaTransition(panorama_index_pair[0], panorama_index_pair[1], weight_pair[0]);
}

void MainWidget::RenderPanoramaTransition() {
  RenderPanoramaTransition(navigation.GetCameraPanorama().start_index,
                           navigation.GetCameraPanorama().end_index,
                           navigation.ProgressInverse());
}

void MainWidget::RenderPanoramaTransition(const int start_index,
                                          const int end_index,
                                          const double start_weight) {
    // Render the source pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);    
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[start_index].Render(1.0);

  // Render the target pano.
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[1]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  panorama_renderers[end_index].Render(1.0);

  // Blend the two.
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
                          static_cast<float>(start_weight));
  
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

void MainWidget::RenderPolygon(const int room_not_rendered,
                               const double alpha,
                               const double height_adjustment,
                               const bool depth_order_height_adjustment,
                               const int room_highlighted) {
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);

  const bool kNotRenderLabel = false;
  polygon_renderer.RenderWallAll(navigation.GetCenter(),
                                 alpha,
                                 height_adjustment,
                                 depth_order_height_adjustment,
                                 room_not_rendered,
                                 room_highlighted,
                                 kNotRenderLabel);

  //polygon_renderer.RenderWireframeAll(alpha);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
}

void MainWidget::RenderTexturedPolygon(const double alpha) {
  glEnable(GL_TEXTURE_2D);
  // glEnable(GL_BLEND);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);

  polygon_renderer.RenderTextureMappedRooms(alpha);

  //polygon_renderer.RenderWireframeAll(alpha);

  // glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  // glDisable(GL_BLEND);
  glDisable(GL_TEXTURE_2D);
}

void MainWidget::RenderPolygonLabels(const int room_not_rendered,
                                     const double height_adjustment,
                                     const bool depth_order_height_adjustment) {
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);    
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_CULL_FACE);

  const bool kRenderLabel = true;
  polygon_renderer.RenderWallAll(navigation.GetCenter(),
                                 1.0,
                                 height_adjustment,
                                 depth_order_height_adjustment,
                                 room_not_rendered,
                                 -1,
                                 kRenderLabel);

  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MainWidget::RenderThumbnail(const double alpha, const int room_highlighted) {
  Vector2i render_pos(mouseMovePosition[0], mouseMovePosition[1]);
  // Decide to draw where. Either right or left. At default right. When too close left.
  const double kMarginRatio = 0.05;
  if (render_pos[0] < viewport[2] * 0.85)
    render_pos[0] += static_cast<int>(round(viewport[2] * kMarginRatio));
  else
    render_pos[0] -= static_cast<int>(round(viewport[2] * (kMarginRatio + PanelRenderer::kWidthRatio / 2)));
  const double kScale = 1.0;
  panel_renderer.RenderThumbnail(alpha,
                                 room_highlighted,
                                 render_pos,
                                 Vector3d(0.4, 0.3, 0.3),
                                 kScale,
                                 this);
}

void MainWidget::RenderAllThumbnails(const double alpha, const int room_highlighted) {
  // Make thumbnails smaller when rendering everything.
  const double kScale = 0.5;

  const Floorplan& floorplan = polygon_renderer.GetFloorplan();
  const int num_room = floorplan.GetNumRooms();
  vector<Vector2i> room_centers(num_room);
  for (int room = 0; room < num_room; ++room) {
    const Vector3d& center = polygon_renderer.GetRoomCenterFloorGlobal(room);
    GLdouble u, v, w;
    gluProject(center[0], center[1], center[2], modelview, projection, viewport, &u, &v, &w);
    room_centers[room][0] = static_cast<int>(round(u));
    room_centers[room][1] = static_cast<int>(round(v));
  }

  vector<pair<int, int> > render_order;
  for (int room = 0; room < (int)room_centers.size(); ++room)
    render_order.push_back(make_pair(room_centers[room][1], room));
  sort(render_order.rbegin(), render_order.rend());
  
  for (int i = 0; i < (int)render_order.size(); ++i) {
    const int room = render_order[i].second;
    const int offset_x = static_cast<int>(round(kScale * PanelRenderer::kWidthRatio * viewport[2] / 2.0));
    const int offset_y = offset_x * panel_renderer.GetRoomThumbnail(room).height() /
      panel_renderer.GetRoomThumbnail(room).width();
    
    if (room == room_highlighted)
      panel_renderer.RenderThumbnail(alpha,
                                     room,
                                     Vector2i(room_centers[room][0] - offset_x,
                                              viewport[3] - room_centers[room][1] - offset_y),
                                     Vector3d(1, 1, 1),
                                     kScale,
                                     this);
    else
      panel_renderer.RenderThumbnail(alpha,
                                     room,
                                     Vector2i(room_centers[room][0] - offset_x,
                                              viewport[3] - room_centers[room][1] - offset_y),
                                     Vector3d(0.4, 0.3, 0.3),
                                     kScale,
                                     this);
  }    
  /*
  Vector2i render_pos(mouseMovePosition[0], mouseMovePosition[1]);
  // Decide to draw where. Either right or left. At default right. When too close left.
  const double kMarginRatio = 0.05;
  if (render_pos[0] < viewport[2] * 0.85)
    render_pos[0] += static_cast<int>(round(viewport[2] * kMarginRatio));
  else
    render_pos[0] -= static_cast<int>(round(viewport[2] * (kMarginRatio + PanelRenderer::kWidthRatio / 2)));

  panel_renderer.RenderThumbnail(alpha, room_highlighted, render_pos, this);
  */
}

void MainWidget::paintGL() {  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  SetMatrices();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  const bool kDepthOrderHeightAdjustment = true;
  const bool kUniformHeightAdjustment = false;
  switch (navigation.GetCameraStatus()) {
  case kPanorama: {
    // When mouse is moving, this is called in every frame, and
    // becomes slow. So, we do this only when the mouse is not
    // moving. Interaction
    if (fresh_screen_for_panorama && !mouse_down) {
      RenderPolygonLabels(panorama_to_room[navigation.GetCameraPanorama().start_index],
                          HeightAdjustment(),
                          kDepthOrderHeightAdjustment);
      fresh_screen_for_panorama = false;
    }
    
    if (!RightAfterSimpleClick(0.0)) {
      RenderPanorama(1.0);
      // RenderFloorplan();
    } else {
      const double alpha = Fade();
      RenderPanorama(1.0 - alpha * 0.7);
      RenderFloorplan(alpha / 2.0);

      // Checks if any room should be highlighted.
      int room_highlighted = -1;
      if (!mouse_down)
        room_highlighted = FindRoomHighlighted(Vector2i(mouseMovePosition[0],
                                                        mouseMovePosition[1]));
      RenderPolygon(panorama_to_room[navigation.GetCameraPanorama().start_index],
                    alpha / 2.0, HeightAdjustment(), kDepthOrderHeightAdjustment, room_highlighted);
      RenderThumbnail(1.0, room_highlighted);
    }
    break;
  }
  case kPanoramaTransition: {
    RenderPanoramaTransition();
    break;
  }
  case kAir: {
    if (fresh_screen_for_air && !mouse_down) {
      RenderPolygonLabels(-1, HeightAdjustment(), kUniformHeightAdjustment);
      fresh_screen_for_air = false;
    }
        
    if (!RightAfterSimpleClick(0.0)) {
      /*
      RenderFloorplan(1.0);
      const double kNoHeightAdjustment = 0.0;
      RenderPolygon(-1, 1.0 / 3.0, kNoHeightAdjustment, kUniformHeightAdjustment, -1);
      */
      //????
      RenderTexturedPolygon(1.0);
    } else {
      const double alpha = Fade();
      RenderFloorplan(alpha / 2.0);

      int room_highlighted = -1;
      if (!mouse_down)
        room_highlighted = FindRoomHighlighted(Vector2i(mouseMovePosition[0],
                                                        mouseMovePosition[1]));
      RenderPolygon(-1, 1.0 / 3.0, HeightAdjustment(), kUniformHeightAdjustment, room_highlighted);
      RenderAllThumbnails(alpha, room_highlighted);
    }
    break;
  }
  case kAirTransition: {
    //if (RightAfterSimpleClick(0.0)) {
    //const double alpha = Fade();
    RenderFloorplan(1.0);
      //}
    const double kNoHeightAdjustment = 0.0;
    RenderPolygon(-1, 1.0 / 3.0, kNoHeightAdjustment, kUniformHeightAdjustment, -1);
    break;
  }
  case kPanoramaToAirTransition: {
    const double alpha = navigation.ProgressInverse();
    RenderPanorama(alpha);
    RenderFloorplan(1.0 - alpha);
    const double kNoHeightAdjustment = 0.0;
    RenderPolygon(-1, 1.0 / 3.0, kNoHeightAdjustment, kUniformHeightAdjustment, -1);
    break;
  }
  case kAirToPanoramaTransition: {
    const double alpha = 1.0 - navigation.ProgressInverse();
    RenderPanorama(alpha);
    RenderFloorplan(1.0 - alpha);
    const double kNoHeightAdjustment = 0.0;
    RenderPolygon(-1, 1.0 / 3.0, kNoHeightAdjustment, kUniformHeightAdjustment, -1);
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

int MainWidget::FindRoomHighlighted(const Eigen::Vector2i& pixel) {
  unsigned char data;
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);
  glReadPixels(pixel[0], viewport[3] - pixel[1], 1, 1, GL_BLUE, GL_UNSIGNED_BYTE, &data);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  
  return static_cast<int>(data) - 1;
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

        FindPanoramaPath(navigation.GetCameraPanorama().start_index,
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
      navigation.RotatePanorama(diff.x(), diff.y());
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
  case kAirToPanoramaTransition:
  case kPanoramaTour: {
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

bool MainWidget::RightAfterSimpleClick(const double margin) const {
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
  const double kSpeed = 20.0;
  if (progress < 1.0 / kSpeed) {
    if (offset > kFadeInSeconds)
      return 1.0;
    else
      return kSpeed * progress;
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
  return min(1.0, 0.2 + 10.0 * max(0.0, progress - 0.2));
}

void MainWidget::SetPanoramaToRoom() {
  const Floorplan& floorplan = polygon_renderer.GetFloorplan();
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    const Vector3d global_center = panorama_renderers[p].GetCenter();
    const Vector3d floorplan_center = floorplan_to_global.transpose() * global_center;
    const Vector2d floorplan_center2(floorplan_center[0], floorplan_center[1]);
           
    int room_id = -1;
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      if (IsInside(floorplan, room, floorplan_center2)) {
        room_id = room;
        break;
      }
    }
    panorama_to_room[p] = room_id;
  }
}

void MainWidget::SetRoomToPanorama() {
  const Floorplan& floorplan = polygon_renderer.GetFloorplan();
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const Vector2d room_center = polygon_renderer.GetRoomCenter(room);
    
    // Find the closest panorama.
    int best_panorama = -1;
    double best_distance = 0.0;
    for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
      const Vector3d global_center = panorama_renderers[p].GetCenter();
      const Vector3d floorplan_center = floorplan_to_global.transpose() * global_center;
      const Vector2d panorama_center(floorplan_center[0], floorplan_center[1]);
      const double distance = (room_center - panorama_center).norm();
      if (best_panorama == -1 || distance < best_distance) {
        best_panorama = p;
        best_distance = distance;
      }
    }
    room_to_panorama[room] = best_panorama;
  }
}

void MainWidget::SetPanoramaDistanceTable() {
  panorama_distance_table.clear();
  panorama_distance_table.resize(panorama_renderers.size());
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    panorama_distance_table[p].resize(panorama_renderers.size(), -1);
  }
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    for (int q = p + 1; q < (int)panorama_renderers.size(); ++q) {
      panorama_distance_table[p][q] = panorama_distance_table[q][p] =
        (ComputePanoramaDistance(p, q) + ComputePanoramaDistance(q, p)) / 2.0;
    }
  }
}

double MainWidget::ComputePanoramaDistance(const int lhs, const int rhs) const {
  const Vector2d lhs_on_rhs_depth_image =
    panorama_renderers[rhs].RGBToDepth(panorama_renderers[rhs].Project(panorama_renderers[lhs].GetCenter()));
  // Search in some radius.
  const int wradius = panorama_renderers[rhs].DepthWidth() / 20;
  const int hradius = panorama_renderers[rhs].DepthHeight() / 20;
  const double distance = (panorama_renderers[lhs].GetCenter() -
                           panorama_renderers[rhs].GetCenter()).norm();

  int connected = 0;
  int occluded = 0;
  
  const int depth_width              = panorama_renderers[rhs].DepthWidth();
  const int depth_height             = panorama_renderers[rhs].DepthHeight();

  const vector<Vector3d>& depth_mesh = panorama_renderers[rhs].DepthMesh();
  // Only look at the top half.
  for (int j = -hradius; j <= 0; ++j) {
    const int ytmp = static_cast<int>(round(lhs_on_rhs_depth_image[1])) + j;
    if (ytmp < 0 || depth_height <= ytmp)
      continue;
    for (int i = -wradius; i <= wradius; ++i) {
      const int xtmp = (static_cast<int>(round(lhs_on_rhs_depth_image[0])) + i + depth_width) % depth_width;
      const Vector3d depth_point = depth_mesh[ytmp * depth_width + xtmp];
      const double depthmap_distance = (depth_point - panorama_renderers[rhs].GetCenter()).norm();

      if (distance < depthmap_distance)
        ++connected;
      else
        ++occluded;
    }
  }
  if (connected + occluded == 0) {
    cerr << "Impossible in ComputePanoramaDistance" << endl;
    exit (1);
  }
  //cout << '(' << lhs << ',' << rhs << ',' << occluded *1.0/ (connected + occluded) << ") ";
  const double ratio = occluded / static_cast<double>(connected + occluded);
  const double kOffset = 0.0;
  const double kMinScale = 1.0;
  const double kMaxScale = 10.0;

  return distance *
    (kMinScale + (kMaxScale - kMinScale) * max(0.0, ratio - kOffset) /
     (1.0 - kOffset));
}

void MainWidget::FindPanoramaPath(const int start_panorama, const int goal_panorama,
                                  std::vector<int>* indexes) const {
  // Standard shortest path algorithm.
  const double kInvalid = -1.0;
  // Find connectivity information. Distance is put. -1 means not connected.
  vector<pair<double, int> > table(panorama_renderers.size(), pair<double, int>(kInvalid, -1));

  table[start_panorama] = pair<double, int>(0.0, start_panorama);
  // It is enough to repeat iterations at the number of panoramas - 1.
  for (int i = 0; i < (int)panorama_renderers.size() - 1; ++i) {
    // Update the result on panorama p by using q.
    for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
      if (p == start_panorama)
        continue;
      for (int q = 0; q < (int)panorama_renderers.size(); ++q) {
        if (p == q)
          continue;
        // q is not reachable yet.
        if (table[q].first == kInvalid)
          continue;
        
        const double new_distance = table[q].first + panorama_distance_table[q][p];
        if (table[p].first == kInvalid || new_distance < table[p].first) {
          table[p].first = new_distance;
          table[p].second = q;
        }
      }
    }
  }

  if (table[goal_panorama].first == kInvalid) {
    cerr << "Impossible. every node is reachable." << endl;
    exit (1);
  }

  indexes->clear();
  int pindex = goal_panorama;
  indexes->push_back(pindex);
  while (pindex != start_panorama) {
    pindex = table[pindex].second;
    indexes->push_back(pindex);
  }
  reverse(indexes->begin(), indexes->end());

  cout << "Path ";
  for (int i = 0; i < (int)indexes->size(); ++i)
    cout << indexes->at(i) << ' ';
  cout << endl;
}
