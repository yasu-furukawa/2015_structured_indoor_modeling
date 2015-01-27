# Feel free to add more paths here. This should just
# give warnings without errors.

QT       += core gui widgets
CONFIG += c++11
TARGET = viewer
TEMPLATE = app

SOURCES += viewer.cc

#QMAKE_CXXFLAGS += '-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare -I/usr/local/opt/qt'
#QMAKE_CXXFLAGS += '-Wno-c++11-extensions -Wno-gnu-static-float-init -Wno-sign-compare'

qtHaveModule(opengl) {
    QT += opengl

    SOURCES += main_widget.cc \
       configuration.cc \
       navigation.cc \
       floorplan_renderer.cc \
       object_renderer.cc \
       panel_renderer.cc \
       panorama_renderer.cc \
       polygon_renderer.cc \
       ../base/floorplan.cc \
       ../base/point_cloud.cc

    HEADERS += \
        main_widget.h \
        configuration.h \
        navigation.h \
        floorplan_renderer.h \
        object_renderer.h \
        panorama_renderer.h \
        panel_renderer.h \
        polygon_renderer.h \
        ../base/file_io.h \
        ../base/floorplan.h \
        ../base/point_cloud.h

    RESOURCES += \
        shaders.qrc

    unix{
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
#       LIBS += '-L/opt/X11/lib -lGLU -framework OpenGL'
    }
}

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target
