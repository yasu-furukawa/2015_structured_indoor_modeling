QT       += core gui widgets

TARGET = viewer
TEMPLATE = app

SOURCES += viewer.cc

QMAKE_CXXFLAGS += '-Wno-c++11-extensions'

qtHaveModule(opengl) {
    QT += opengl

    SOURCES += main_widget.cc \
       configuration.cc \
       panorama.cc \
       navigation.cc \
       floorplan_renderer.cc \
       ../floorplan/floorplan.cc

    HEADERS += \
        main_widget.h \
        configuration.h \
        panorama.h \
        navigation.h \
        ../calibration/file_io.h \
        ../floorplan/floorplan.h

    RESOURCES += \
        shaders.qrc

    INCLUDEPATH += '/Users/furukawa/Google Drive/research/code/'
#    INCLUDEPATH += '/opt/X11/include'

     

     LIBS += '-F/Users/furukawa/Qt/5.3/clang_64/lib/QtOpenGL.framework/Versions/5/'
#    LIBS += '-L/opt/X11/lib -lGLU -framework OpenGL'
}

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target
