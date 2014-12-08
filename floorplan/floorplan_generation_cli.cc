#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#include "../calibration/file_io.h"
#include "floorplan.h"
#include "floorplan_util.h"

using namespace std;

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
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " input_data_directory output_prefix" << endl;
    exit (1);
  }

  file_io::FileIO file_io(argv[1]);
  LineFloorplan line_floorplan;
  {
    // char buffer[1024];
    // sprintf(buffer, "%s/floorplan.txt", argv[1]);
    ifstream ifstr;
    ifstr.open(file_io.GetLineFloorplan().c_str());
    ifstr >> line_floorplan;
    ifstr.close();
  }
  Floorplan floorplan;
  const double kOutwardThickness = 50;
  const double kInwardThickness = 0;
  ConvertLineFloorplanToFloorplan(line_floorplan,
                                  kOutwardThickness,
                                  kInwardThickness,
                                  &floorplan);
  // Saving svg.
  {
    char buffer0[1024], buffer1[1024];
    if (argc < 3) {
      sprintf(buffer0, "%s/floorplan.svg", argv[1]);
      sprintf(buffer1, "%s/triangulation.svg", argv[1]);
    } else {
      sprintf(buffer0, "%s/%sfloorplan.svg", argv[1], argv[2]);
      sprintf(buffer1, "%s/%striangulation.svg", argv[1], argv[2]);
    }
    FloorplanToSVG(floorplan, buffer0, buffer1);
  }
  
  // Saving data.
  {
    char buffer[1024];
    if (argc < 3)
      sprintf(buffer, "%s/floorplan.dat", argv[1]);
    else
      sprintf(buffer, "%s/%sfloorplan.dat", argv[1], argv[2]);
    ofstream ofstr;
    ofstr.open(buffer);
    ofstr << floorplan;
    ofstr.close();
  }
  
  return 0;
}














  
  
//   {
//       Polygon_2 P;
//       P.push_back (Point_2 (0,0));
//       P.push_back (Point_2 (5,0));
//       P.push_back (Point_2 (3.5,1.5));
//       P.push_back (Point_2 (2.5,0.5));
//       P.push_back (Point_2 (1.5,1.5));

//       Polygon_2 Q;
//       Q.push_back(Point_2 (0, 2));
//       Q.push_back(Point_2 (1.5, 0.5));
//       Q.push_back(Point_2 (2.5, 1.5));
//       Q.push_back(Point_2 (3.5, 0.5));
//       Q.push_back(Point_2 (5, 2));

//       Polygon_2 P2;
//       P2.push_back (Point_2 (10,0));
//       P2.push_back (Point_2 (15,0));
//       P2.push_back (Point_2 (13.5,1.5));
//       P2.push_back (Point_2 (12.5,0.5));
//       P2.push_back (Point_2 (11.5,1.5));

//       Polygon_2 Q2;
//       Q2.push_back(Point_2 (10, 2));
//       Q2.push_back(Point_2 (11.5, 0.5));
//       Q2.push_back(Point_2 (12.5, 1.5));
//       Q2.push_back(Point_2 (13.5, 0.5));
//       Q2.push_back(Point_2 (15, 2));

//       Polygon_set_2 S;
//       S.insert(P);
//       S.join(Q);
//       S.join(P2);
//       S.join(Q2);

//       list<Polygon_with_holes_2> res;
//       list<Polygon_with_holes_2>::const_iterator it;
//       cout << S.number_of_polygons_with_holes() << " components" << endl;
//       S.polygons_with_holes(std::back_inserter(res));
//       for (it = res.begin(); it != res.end(); ++it) {
//         cout << "--> ";
//         print_polygon_with_holes(*it);
//       }
      
//       /*
      
      
//       Polygon_with_holes_2 union0, union1, union2;
//       CGAL::join(P, P2, union0);
//       print_polygon_with_holes(union0);
//       */
//       /*
//         CGAL::join(union0, P2, union1);
//         CGAL::join(union1, Q2, union2);
//         print_polygon_with_holes(union2);
//       */
//       exit (1);
//   }
//   exit (1);

  
//   Polygon_2 P;
//   P.push_back (Point_2 (1,0));
//   P.push_back (Point_2 (1,3));
//   P.push_back (Point_2 (0,3));
//   P.push_back (Point_2 (0,0));

//   Polygon_2 Q;
//   Q.push_back(Point_2 (3, 2));
//   Q.push_back(Point_2 (3, 3));
//   Q.push_back(Point_2 (0, 3));
//   Q.push_back(Point_2 (0, 2));

//   Polygon_2 R;
//   R.push_back(Point_2 (2, 0));
//   R.push_back(Point_2 (3, 0));
//   R.push_back(Point_2 (3, 3));
//   R.push_back(Point_2 (2, 3));

//   Polygon_2 S;
//   S.push_back(Point_2 (3, 0));
//   S.push_back(Point_2 (3, 1));
//   S.push_back(Point_2 (0, 1));
//   S.push_back(Point_2 (0, 0));

  
//   Polygon_with_holes_2 union0;
//   Polygon_with_holes_2 union1, union2;
//   if (CGAL::join(P, Q, union0)) {
//     print_polygon_with_holes(union0);

//     if (CGAL::join(union0, R, union1)) {
//       print_polygon_with_holes(union1);
      
//       if (CGAL::join(union1, S, union2)) {
//         print_polygon_with_holes(union2);
//         cout << union2 << endl;

//       }
//     }
//   }
//   /*

  
//   if (CGAL::join (P, Q, unionR)) {
//     std::cout << "The union: ";
//     print_polygon_with_holes (unionR);
//   } else
//     std::cout << "P and Q are disjoint and their union is trivial."
// 	      << std::endl;
//   std::cout << std::endl;
//   */
  
//   return 0;
// }
