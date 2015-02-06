# Feel free to add more paths here. This should just
# give warnings without errors.

QT       += core gui widgets
CONFIG += c++11
TARGET = viewer
TEMPLATE = app

SOURCES += viewer.cc

macx{
QMAKE_CXXFLAGS += '-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare -Wno-overloaded-virtual'
}

qtHaveModule(opengl) {
    QT += opengl

    SOURCES += \
       main_widget.cc \
       main_widget_render.cc \
       main_widget_util.cc \
       navigation.cc \
       floorplan_renderer.cc \
       object_renderer.cc \
       panel_renderer.cc \
       panorama_renderer.cc \
       polygon_renderer.cc \
       ../base/floorplan.cc \
       ../base/panorama.cc \
       ../base/point_cloud.cc

    HEADERS += \
        main_widget.h \
        main_widget_render.h \
        main_widget_util.h \
        configuration.h \
        navigation.h \
        floorplan_renderer.h \
        object_renderer.h \
        panorama_renderer.h \
        panel_renderer.h \
        polygon_renderer.h \
        ../base/file_io.h \
        ../base/floorplan.h \
        ../base/panorama.h \
        ../base/point_cloud.h

    RESOURCES += \
        shaders.qrc

    unix:!macx{
        INCLUDEPATH += -I/usr/include
        INCLUDEPATH += -I/usr/local/include
        LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc
        LIBS += -L/usr/lib/x86_64-linux-gnu -lGLU
    }

    macx{
        INCLUDEPATH += '/Users/furukawa/Google_Drive/research/code/'
        INCLUDEPATH += '/usr/local/include/'
#        INCLUDEPATH += '/opt/X11/include'


         LIBS += '-F/Users/furukawa/Qt/5.3/clang_64/lib/QtOpenGL.framework/Versions/5/'
#       LIBS += '-F/usr/local/opt/qt/Frameworks/QtOpenGL.framework/Versions/4/'
        LIBS += '-L/usr/local/lib'
        LIBS += '-lopencv_core.2.4.9'
        LIBS += '-lopencv_imgproc.2.4.9'
        LIBS += '-lopencv_highgui.2.4.9'
#       LIBS += '-L/opt/X11/lib -lGLU -framework OpenGL'
    }

    win32{
        LIBS += -LC:/opencv/build/x64/vc12/lib -lopencv_core2410 -lopencv_imgproc2410 -lopencv_highgui2410 -lopencv_core2410d -lopencv_imgproc2410d -lopencv_highgui2410d
        INCLUDEPATH +=  C:/opencv/build/include
        #INCLUDEPATH += $$PWD/../../../lib/eigen3
        INCLUDEPATH += C:/Eigen3.2.2/
    }
}

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target
