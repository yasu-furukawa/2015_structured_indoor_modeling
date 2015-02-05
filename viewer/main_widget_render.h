#pragma once

#include <fstream>
#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>
#include <QVector2D>

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <GL/GLU.h>
#else
#include <OpenGL/glu.h>
#endif

#include <opencv2/opencv.hpp>

namespace structured_indoor_modeling {

class FloorplanRenderer;
class Navigation;
class PanoramaRenderer;
class PolygonRenderer;
class ObjectRenderer;

void RenderFloorplan(const FloorplanRenderer& floorplan_renderer,
                     const double alpha);

void RenderPanorama(const Navigation& navigation,
                    const std::vector<PanoramaRenderer>& panorama_renderers,
                    const double alpha);

void RenderObjects(ObjectRenderer& object_renderer, const double alpha);
void RenderPolygon(const Navigation& navigation,
                   PolygonRenderer& polygon_renderer,
                   const int room_not_rendered,
                   const double alpha,
                   const double height_adjustment,
                   const bool depth_order_height_adjustment,
                   const int room_highlighted);

void RenderTexturedPolygon(const PolygonRenderer& polygon_renderer,
                           const double alpha);

void RenderPolygonLabels(const Navigation& navigation,
                         PolygonRenderer& polygon_renderer,
                         const GLuint frameids[],
                         const int room_not_rendered,
                         const double height_adjustment,
                         const bool depth_order_height_adjustment);

void RenderThumbnail(PanelRenderer& panel_renderer,
                     const QVector2D mouseMovePosition,
                     const GLint viewport[],
                     const double alpha,
                     const int room_highlighted,
                     QGLWidget* qgl_widget);

void RenderAllThumbnails(PanelRenderer& panel_renderer,
                         const Floorplan& floorplan,
                         const GLint viewport[],
                         const GLdouble modelview[],
                         const GLdouble projection[],
                         const double alpha,
                         const int room_highlighted,
                         QGLWidget* qgl_widget);
     
void RenderPanoramaTransition(QOpenGLShaderProgram& program,
                              const Navigation& navigation,
                              std::vector<PanoramaRenderer>& panorama_renderers,
                              const GLuint frameids[],
                              const GLuint texids[],
                              const int width,
                              const int height,
                              const int start_index,
                              const int end_index,
                              const double start_weight);

void BlendFrames(QOpenGLShaderProgram& program,
                 const GLuint texids[], const int width, const int height,
                 const double weight, const int divide_by_alpha_mode);

void RenderPanoramaTour(QOpenGLShaderProgram& program,
                        const Navigation& navigation,
                        std::vector<PanoramaRenderer>& panorama_renderers,
                        const GLuint frameids[],
                        const GLuint texids[],
                        const int width,
                        const int height);

void RenderPanoramaToAirTransition(QOpenGLShaderProgram& program,
                                   const Navigation& navigation,
                                   const std::vector<PanoramaRenderer>& panorama_renderers,
                                   const PolygonRenderer& polygon_renderer,
                                   ObjectRenderer& object_renderer,
                                   const GLuint frameids[],
                                   const GLuint texids[],
                                   const int width,
                                   const int height,
                                   const bool flip = false);

void RenderPanoramaToFloorplanTransition(QOpenGLShaderProgram& program,
                                         const Navigation& navigation,
                                         const std::vector<PanoramaRenderer>& panorama_renderers,
                                         const FloorplanRenderer& floorplan_renderer,
                                         const GLuint frameids[],
                                         const GLuint texids[],
                                         const int width,
                                         const int height,                                         
                                         const bool flip = false);

void RenderAirToFloorplanTransition(QOpenGLShaderProgram& program,
                                    const Navigation& navigation,
                                    const PolygonRenderer& polygon_renderer,
                                    ObjectRenderer& object_renderer,
                                    const FloorplanRenderer& floorplan_renderer,
                                    const GLuint frameids[],
                                    const GLuint texids[],
                                    const int width,
                                    const int height,                                    
                                    const bool flip = false);
 
}  // namespace structured_indoor_modeling
