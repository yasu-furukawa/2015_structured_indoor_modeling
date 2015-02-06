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
#include <windows.h>
#include <GL/glu.h>
//#ifndef __glew_h__
//#include <GL/glew.h>
//#include <GL/glext.h>
//#endif
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

 
}  // namespace structured_indoor_modeling
