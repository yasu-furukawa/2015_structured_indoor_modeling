#ifndef MAIN_WIDGET_H__
#define MAIN_WIDGET_H__

#include <QGLWidget>
//#include <QGLFunctions>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QTime>

#include <map>
#include <string>
#include <vector>

#include "../base/file_io.h"
#include "configuration.h"
#include "navigation.h"
#include "floorplan_renderer.h"
#include "object_renderer.h"
#include "panel_renderer.h"
#include "panorama_renderer.h"
#include "polygon_renderer.h"

namespace structured_indoor_modeling {
  
class MainWidget : public QGLWidget, protected QOpenGLFunctions {
Q_OBJECT
public:
    explicit MainWidget(const Configuration& configuration,
                      QWidget *parent = 0);
    ~MainWidget();

protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void wheelEvent(QWheelEvent* e);
    
    void timerEvent(QTimerEvent *e);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    FileIO file_io;
    //----------------------------------------------------------------------
    // Core data.
    Floorplan floorplan;
    std::vector<Panorama> panoramas;  // No image data loaded.
    
    //----------------------------------------------------------------------
    // Renderers.
    std::vector<PanoramaRenderer> panorama_renderers;
    ObjectRenderer object_renderer;
    PolygonRenderer polygon_renderer;
    FloorplanRenderer floorplan_renderer;
    PanelRenderer panel_renderer;
    // Navigation knows the entire state of the viewer.
    Navigation navigation;

    //----------------------------------------------------------------------
    // GL resources.
    GLuint frameids[2];
    GLuint texids[2];
    GLuint renderids[2];
    int current_width;
    int current_height;

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    // GUI states.
    double prev_animation_linear;
    double prev_animation_trapezoid;    
    bool fresh_screen_for_panorama;
    bool fresh_screen_for_air;
    bool fresh_screen_for_floorplan;
    double simple_click_time_offset_by_move;
    
    QBasicTimer timer;
    QVector2D mousePressPosition;
    QVector2D mouseMovePosition;

    QOpenGLShaderProgram blend_program;

    QTime simple_click_time;
    QTime double_click_time;
    bool mouse_down;

    void FreeResources();
    void AllocateResources();
    void SetMatrices();


    void InitPanoramasPanoramaRenderers();
    // void RenderQuad(const double alpha);
    void InitializeShaders();
   
    // Keep rendering after no action for a while.
    bool RightAfterSimpleClick(const double margin) const;

    // After a single click, where we are between kFadeInSeconds and
    // kFadeOutSeconds.
    // double Progress();
    double AnimationTrapezoid();
    double AnimationLinear();

    std::map<int, int> panorama_to_room;
    std::map<int, int> room_to_panorama;
    std::vector<std::vector<double> > panorama_distance_table;
    
    static const double kRenderMargin;
    static const double kFadeInSeconds;
    static const double kFadeOutSeconds;
    static const Eigen::Vector3f kBackgroundColor;

    //----------------------------------------------------------------------
    // Paint functions.
    void PaintPanorama();
    void PaintAir();
    void PaintFloorplan();
    
    //----------------------------------------------------------------------
    // Render functions.
    void RenderFloorplan(const double alpha,
                         const bool emphasize,
                         const double height_adjustment);
    void RenderPanorama(const double alpha);
    void RenderObjects(const double alpha);
    void RenderPolygon(const int room_not_rendered,
                       const double alpha,
                       const double height_adjustment,
                       const bool depth_order_height_adjustment,
                       const int room_highlighted);
    void RenderTexturedPolygon(const double alpha);
    void RenderPolygonLabels(const int room_not_rendered,
                             const double height_adjustment,
                             const bool depth_order_height_adjustment);
    void RenderFloorplanLabels();
    void RenderThumbnail(const double alpha,
                         const int room_highlighted,
                         QGLWidget* qgl_widget);
    void RenderAllThumbnails(const double alpha,
                             const int room_highlighted,
                             QGLWidget* qgl_widget);
    void RenderAllRoomNames(const double alpha,
                            QGLWidget* qgl_widget);
    
    void RenderPanoramaTransition(const int start_index,
                                  const int end_index,
                                  const double start_weight);
    void BlendFrames(const double weight, const int divide_by_alpha_mode);
    void RenderPanoramaTour();    
    void RenderPanoramaToAirTransition(const bool flip = false);
    void RenderPanoramaToFloorplanTransition(const bool flip = false);
    void RenderAirToFloorplanTransition(const bool flip = false);
    int FindRoomHighlighted(const Eigen::Vector2i& pixel);    

    void ClearDisplay();
    void ClearDisplayWithWhite();
};

}  // namespace structured_indoor_modeling

#endif // MAIN_WIDGET_H__
