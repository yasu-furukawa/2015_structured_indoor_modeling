#pragma once

#include <fstream>
#include <iostream>
#include <locale.h>
#include <math.h>
#include <Eigen/Dense>

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

class Floorplan;
class PanoramaRenderer;

int FindPanoramaFromAirFloorplanClick(const std::vector<PanoramaRenderer>& panorama_renderers,
                                      const Eigen::Vector2d& pixel,
                                      const GLint viewport[],
                                      const GLdouble modelview[],
                                      const GLdouble projection[]);

/*
double ProgressFunction(const double elapsed,
                        const double offset,
                        const double fade_in_seconds,
                        const double fade_out_seconds);
*/

double AnimationTrapezoidUtil(const double elapsed,
                              const double offset,
                              const double fade_in_seconds,
                              const double fade_out_seconds);

double AnimationLinearUtil(const double elapsed,
                           const double offset,
                           const double fade_in_seconds,
                           const double fade_out_seconds);

void SetPanoramaToRoom(const Floorplan& floorplan,
                       const std::vector<PanoramaRenderer>& panorama_renderers,
                       std::map<int, int>* panorama_to_room);

 void SetRoomToPanorama(const Floorplan& floorplan,
                        const std::vector<PanoramaRenderer>& panorama_renderers,
                        std::map<int, int>* room_to_panorama);

void SetPanoramaDistanceTable(const std::vector<PanoramaRenderer>& panorama_renderers,
                              std::vector<std::vector<double> >* panorama_distance_table);

double ComputePanoramaDistance(const PanoramaRenderer& lhs,
                               const PanoramaRenderer& rhs);

void FindPanoramaPath(const std::vector<PanoramaRenderer>& panorama_renderers,
                      const std::vector<std::vector<double> >& panorama_distance_table,
                      const int start_panorama,
                      const int goal_panorama,
                      std::vector<int>* indexes);

}  // namespace structured_indoor_modeling
