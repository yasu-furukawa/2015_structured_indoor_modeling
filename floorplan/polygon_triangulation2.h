#ifndef CGAL_POLYGON_TRIANGULATION2_H__
#define CGAL_POLYGON_TRIANGULATION2_H__

#include <vector>
#include <Eigen/Dense>

#include <CGAL/basic.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>


namespace Cgal {
  typedef float Coord_type;
  typedef CGAL::Point_2<CGAL::Cartesian<Coord_type> > Point_2;
  typedef CGAL::Delaunay_triangulation_2<CGAL::Cartesian<Coord_type> > Delaunay2;
  
  typedef CGAL::Constrained_Delaunay_triangulation_2<CGAL::Cartesian<Coord_type> > CDelaunay2;
  
  void Triangulate(const std::vector<Eigen::Vector2d>& points,
                   std::vector<Eigen::Vector3i>* triangles);

};

#endif // CGAL_POLYGON_TRIANGULATION2_H__
