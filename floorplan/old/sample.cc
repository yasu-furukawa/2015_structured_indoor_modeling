#include <fstream>
#include <iostream>

#include <vector>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/General_polygon_with_holes_2.h>

#include <list>

using namespace std;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef CGAL::General_polygon_with_holes_2<Kernel>         General_polygon_with_holes_2;
// typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;


template<class Kernel, class Container>
void print_polygon(const CGAL::Polygon_2<Kernel, Container>& P) {
  typename CGAL::Polygon_2<Kernel, Container>::Vertex_const_iterator vit;

  cout << "[ " << P.size() << " vertices: ";
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    cout << " (" << *vit << ')';
  cout << " ]" << endl;
}


template<class Kernel, class Container>
void print_polygon_with_holes(const CGAL::Polygon_with_holes_2<Kernel, Container> & pwh)
{
  if (! pwh.is_unbounded()) {
    std::cout << "{ Outer boundary = "; 
    print_polygon (pwh.outer_boundary());
  } else
    std::cout << "{ Unbounded polygon." << std::endl;
  typename CGAL::Polygon_with_holes_2<Kernel,Container>::Hole_const_iterator hit;
  unsigned int k = 1;
  std::cout << " " << pwh.number_of_holes() << " holes:" << std::endl;
  for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit, ++k) {
    std::cout << " Hole #" << k << " = ";
    print_polygon (*hit);
  }
  std::cout << " }" << std::endl;
}

// Take room outline 1D polygons (with doors?), thicken walls, and generate 2D polygons.
int main(int argc, char* argv[]) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " inptu_data output_data" << endl;
    exit (1);
  }

  {
      Polygon_2 P;
      P.push_back (Point_2 (0,0));
      P.push_back (Point_2 (5,0));
      P.push_back (Point_2 (3.5,1.5));
      P.push_back (Point_2 (2.5,0.5));
      P.push_back (Point_2 (1.5,1.5));

      Polygon_2 Q;
      Q.push_back(Point_2 (0, 2));
      Q.push_back(Point_2 (1.5, 0.5));
      Q.push_back(Point_2 (2.5, 1.5));
      Q.push_back(Point_2 (3.5, 0.5));
      Q.push_back(Point_2 (5, 2));

      Polygon_with_holes_2 unionR;
      if (CGAL::join(P, Q, unionR)) {
        print_polygon_with_holes(unionR);
        //        exit (1);
      }
  }

  
  Polygon_2 P;
  P.push_back (Point_2 (1,0));
  P.push_back (Point_2 (1,3));
  P.push_back (Point_2 (0,3));
  P.push_back (Point_2 (0,0));

  Polygon_2 Q;
  Q.push_back(Point_2 (3, 2));
  Q.push_back(Point_2 (3, 3));
  Q.push_back(Point_2 (0, 3));
  Q.push_back(Point_2 (0, 2));

  Polygon_2 R;
  R.push_back(Point_2 (2, 0));
  R.push_back(Point_2 (3, 0));
  R.push_back(Point_2 (3, 3));
  R.push_back(Point_2 (2, 3));

  Polygon_2 S;
  S.push_back(Point_2 (3, 0));
  S.push_back(Point_2 (3, 1));
  S.push_back(Point_2 (0, 1));
  S.push_back(Point_2 (0, 0));

  
  Polygon_with_holes_2 union0;
  Polygon_with_holes_2 union1, union2;
  if (CGAL::join(P, Q, union0)) {
    print_polygon_with_holes(union0);

    if (CGAL::join(union0, R, union1)) {
      print_polygon_with_holes(union1);
      
      if (CGAL::join(union1, S, union2)) {
        print_polygon_with_holes(union2);
        cout << union2 << endl;

      }
    }
  }
  /*

  
  if (CGAL::join (P, Q, unionR)) {
    std::cout << "The union: ";
    print_polygon_with_holes (unionR);
  } else
    std::cout << "P and Q are disjoint and their union is trivial."
	      << std::endl;
  std::cout << std::endl;
  */
  
  return 0;
}
