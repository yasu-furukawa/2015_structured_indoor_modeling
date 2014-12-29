#ifndef MAIN_WIDGET_H__
#define MAIN_WIDGET_H__

#include <QGLWidget>
#include <QGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QTime>

#include <map>
#include <string>
#include <vector>

#include "configuration.h"
#include "navigation.h"
#include "floorplan_renderer.h"
#include "panel_renderer.h"
#include "panorama_renderer.h"
#include "polygon_renderer.h"

class MainWidget : public QGLWidget, protected QGLFunctions {
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
    
    void timerEvent(QTimerEvent *e);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    // Indoor data.
    const Configuration configuration;
    std::vector<PanoramaRenderer> panorama_renderers;
    PolygonRenderer polygon_renderer;
    FloorplanRenderer floorplan_renderer;
    PanelRenderer panel_renderer;
    Navigation navigation;

    // Resources.
    GLuint frameids[2];
    GLuint texids[2];
    GLuint renderids[2];
    int current_width;
    int current_height;

    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    double prev_height_adjustment;
    bool fresh_screen_for_panorama;
    bool fresh_screen_for_air;
    
    QBasicTimer timer;
    QVector2D mousePressPosition;
    QVector2D mouseMovePosition;

    QOpenGLShaderProgram program;

    QTime simple_click_time;
    QTime double_click_time;
    bool mouse_down;

    void FreeResources();
    void AllocateResources();
    void SetMatrices();

    int FindPanoramaFromAirClick(const Eigen::Vector2d& pixel) const;
    int FindRoomHighlighted(const Eigen::Vector2i& pixel);
    
    void RenderFloorplan(const double alpha);
    void RenderPanorama(const double alpha);
    void RenderPanoramaTransition();
    void RenderPanoramaTransition(const int start_index,
                                  const int end_index,
                                  const double start_weight);
    void RenderPolygon(const int room_not_rendered,
                       const double alpha,
                       const double height_adjustment,
                       const bool depth_order_height_adjustment,
                       const int room_highlighted);
    void RenderTexturedPolygon(const double alpha);

    void RenderPolygonLabels(const int room_not_rendered,
                             const double height_adjustment,
                             const bool depth_order_height_adjustment);
    void RenderThumbnail(const double alpha, const int room_highlighted); //, MainWidget* main_widget);
    void RenderAllThumbnails(const double alpha, const int room_highlighted); //, MainWidget* main_widget);
    void RenderPanoramaTour();

    // void RenderQuad(const double alpha);
    void InitializeShaders();
   
    // Keep rendering after no action for a while.
    bool RightAfterSimpleClick(const double margin) const;

    // After a single click, where we are between kFadeInSeconds and
    // kFadeOutSeconds.
    double Progress();
    double Fade();
    double HeightAdjustment();
    static double ProgressFunction(const double elapsed, const double offset);
    static double FadeFunction(const double elapsed, const double offset);
    static double HeightAdjustmentFunction(const double elapsed, const double offset);

    void SetPanoramaToRoom();
    void SetRoomToPanorama();

    double ComputePanoramaDistance(const int lhs, const int rhs) const;
    void SetPanoramaDistanceTable();
    void FindPanoramaPath(const int start_panorama, const int goal_panorama,
                          std::vector<int>* indexes) const;

    double simple_click_time_offset_by_move;

    std::map<int, int> panorama_to_room;
    std::map<int, int> room_to_panorama;

    std::vector<std::vector<double> > panorama_distance_table;
    
    static const double kRenderMargin;
    static const double kFadeInSeconds;
    static const double kFadeOutSeconds;
};

#endif // MAIN_WIDGET_H__
