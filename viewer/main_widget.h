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
    FloorplanRenderer floorplan_renderer;
    std::vector<PanoramaRenderer> panorama_renderers;
    PolygonRenderer polygon_renderer;
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
    
    void RenderFloorplan(const double alpha);
    void RenderPanorama(const double alpha);
    void RenderPanoramaTransition();
    void RenderPolygon(const double alpha, const double height_adjustment);

    // void RenderQuad(const double alpha);
    void InitializeShaders();
   
    // Keep rendering after no action for a while.
    bool RightAfterSimpleClick(const double margin);

    // After a single click, where we are between kFadeInSeconds and
    // kFadeOutSeconds.
    double Progress();
    double Fade();
    double HeightAdjustment();
    static double ProgressFunction(const double elapsed, const double offset);
    static double FadeFunction(const double elapsed, const double offset);
    static double HeightAdjustmentFunction(const double elapsed, const double offset);

    void SetPanoramaToRoom();

    double simple_click_time_offset_by_move;

    std::map<int, int> panorama_to_room;
    
    static const double kRenderMargin = 0.2;
    static const double kFadeInSeconds = 0.2;
    static const double kFadeOutSeconds = 1.5;
};

#endif // MAIN_WIDGET_H__
