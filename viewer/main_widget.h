#ifndef MAIN_WIDGET_H__
#define MAIN_WIDGET_H__

#include <QGLWidget>
#include <QGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>

#include <string>
#include <vector>

#include "configuration.h"
#include "polygon_renderer.h"
#include "navigation.h"
#include "panorama.h"


class MainWidget : public QGLWidget, protected QGLFunctions
{
    Q_OBJECT

public:
    explicit MainWidget(const Configuration& configuration,
                        QWidget *parent = 0);
    ~MainWidget();

protected:
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    
    void timerEvent(QTimerEvent *e);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    // void initShaders();
    // void initTextures();

private:
    // Indoor data.
    const Configuration configuration;
    std::vector<Panorama> panoramas;
    PolygonRenderer polygon_renderer;
    Navigation navigation;

    // Resources.
    GLuint frameids[2];
    GLuint texids[2];
    GLuint renderids[2];
    int current_width;
    int current_height;

    bool init_shader;
    
    QBasicTimer timer;
    QVector2D mousePressPosition;

    QOpenGLShaderProgram program;

    void FreeResources();
    void AllocateResources();
    void SetMatrices();

    void RenderPanorama();
    void RenderPanoramaTransition();
    void RenderQuad(const double alpha);
    void RenderRooms();

    void InitializeShaders();
};

#endif // MAIN_WIDGET_H__
