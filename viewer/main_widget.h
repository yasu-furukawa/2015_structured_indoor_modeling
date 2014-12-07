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

    QTime qtime;
    bool mouse_down;
    bool double_clicked;

    void FreeResources();
    void AllocateResources();
    void SetMatrices();

    int FindPanoramaFromAirClick(const Eigen::Vector2d& pixel) const;
    
    void RenderFloorplan();
    void RenderPanorama();
    void RenderPanoramaTransition();
    void RenderPolygon();

    // void RenderQuad(const double alpha);
    void InitializeShaders();

    static const double kPanoramaFadeInSeconds = 0.1;
    static const double kPanoramaFadeOutSeconds = 1.0;

    static const double kAirFadeInSeconds = 0.1;
    static const double kAirFadeOutSeconds = 1.0;
};

#endif // MAIN_WIDGET_H__
