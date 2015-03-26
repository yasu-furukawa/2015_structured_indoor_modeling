#include "floorplan_util.h"
#include "polygon_triangulation2.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <map>

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;


namespace {
void CreateRectangle(const Vector2d& lhs,
                     const Vector2d& rhs,
                     const double outward_thickness,
                     const double inward_thickness,
                     const double along_thickness,
                     Polygon_2* polygon) {
  const double kEpsilon = 0.01;
  
  Vector2d along = rhs - lhs;
  along.normalize();
  // Outward direction.
  Vector2d ortho(along[1], -along[0]);

  const Vector2d along_epsilon = along * kEpsilon;
  const Vector2d ortho_epsilon = ortho * kEpsilon;
  
  along  *= along_thickness;
  const Vector2d outward = ortho * outward_thickness;
  const Vector2d inward = ortho * inward_thickness;  

  {
    Vector2d tmp = lhs - along - along_epsilon - inward; // ortho_epsilon;
    polygon->push_back(Point_2(tmp[0], tmp[1]));
  }
  {
    Vector2d tmp = lhs - along - along_epsilon + outward; // ortho + ortho_epsilon;
    polygon->push_back(Point_2(tmp[0], tmp[1]));
  }
  {
    Vector2d tmp = rhs + along_epsilon + outward; // rtho + ortho_epsilon;
    polygon->push_back(Point_2(tmp[0], tmp[1]));
  }
  {
    Vector2d tmp = rhs + along_epsilon - inward; // ortho_epsilon;
    polygon->push_back(Point_2(tmp[0], tmp[1]));
  }
}

void SetDoorInformation(const LineFloorplan& line_floorplan,
                        const double outward_thickness,
                        const double inward_thickness,
                        map<pair<int, int>, vector<Polygon_2> >* doorways,
                        map<pair<int, int>, vector<Polygon_2> >* doorwalls) {

  const double kEpsilon = 0.1;
  for (const auto& line_door : line_floorplan.line_doors) {
    // vector<Polygon_2> doorways_polygons, doorwalls_polygons;
    Vector2d along = line_door.second.right - line_door.first.left;
    const double along_length = along.norm();
    along.normalize();
    // Outward direction.
    Vector2d ortho(along[1], -along[0]);
    const Vector2d along_epsilon = along * kEpsilon;

    Vector2d doorway_along;
    if (along_length + inward_thickness > outward_thickness - kEpsilon) {
      doorway_along = along * inward_thickness;
    } else {
      doorway_along = along * (outward_thickness - inward_thickness + kEpsilon);
    }
    along  *= inward_thickness;
    const Vector2d inward = ortho * (inward_thickness + outward_thickness);
    const Vector2d inward_epsilon = ortho * kEpsilon;


    // doorwalls.
    {
      Polygon_2 polygon;
      {
        const Vector2d tmp = line_door.first.left - along + inward;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.right + along + inward;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.right + along;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.first.left - along;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      
      (*doorwalls)[make_pair(line_door.first.room_id, line_door.first.wall_id)].push_back(polygon);
    }
    {
      Polygon_2 polygon;
      {
        const Vector2d tmp = line_door.first.right - along;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.left + along;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.left + along - inward;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.first.right - along - inward;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      
      (*doorwalls)[make_pair(line_door.first.room_id, line_door.first.wall_id)].push_back(polygon);
    }


    //----------------------------------------------------------------------
    // doorways
    {
      Polygon_2 polygon;
      {
        const Vector2d tmp = line_door.first.left - doorway_along - along_epsilon;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.right + doorway_along + along_epsilon;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.second.left + doorway_along + along_epsilon;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      {
        const Vector2d tmp = line_door.first.right - doorway_along - along_epsilon;
        polygon.push_back(Point_2(tmp[0], tmp[1]));
      }
      
      (*doorways)[make_pair(line_door.first.room_id, line_door.first.wall_id)].push_back(polygon);
      (*doorways)[make_pair(line_door.second.room_id, line_door.second.wall_id)].push_back(polygon);
    }
  }
}

void SetFloorplanFloorHeight(const LineFloorplan& line_floorplan,
                             Floorplan* floorplan) {
  floorplan->floor_height = 0.0;
  int denom = 0;
  for (int i = 0; i < line_floorplan.line_rooms.size(); ++i) {
    floorplan->floor_height += line_floorplan.line_rooms[i].floor_height;
    ++denom;
  }
  floorplan->floor_height /= denom;
}

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


  
}  // namespace

void ConvertLineFloorplanToFloorplan(const LineFloorplan& line_floorplan,
                                     const double outward_thickness,
                                     const double inward_thickness,
                                     Floorplan* floorplan) {
  const int room_num = line_floorplan.line_rooms.size();
  // For each room and a wall pair, an array of rectangles to be removed for doors.
  map<pair<int, int>, vector<Polygon_2> > doorways;
  // For each room and a wall pair, additional door walls to be added.
  map<pair<int, int>, vector<Polygon_2> > doorwalls;
  SetDoorInformation(line_floorplan, outward_thickness, inward_thickness, &doorways, &doorwalls);
    
  // First construct the union shape.
  Polygon_set_2 polygon_set;

  int count = 0;
  for (int r = 0; r < line_floorplan.line_rooms.size(); ++r) {
    const auto& line_room = line_floorplan.line_rooms[r];
    // For each line segment construct a rectangle and add.
    for (int w = 0; w < line_room.walls.size(); ++w) {
      const int prev_w = (w - 1 + line_room.walls.size()) % line_room.walls.size();
      const int next_w = (w + 1) % line_room.walls.size();

      // Check the angle.
      Vector2d prev_diff = line_room.walls[w] - line_room.walls[prev_w];
      Vector2d next_diff = line_room.walls[next_w] - line_room.walls[w];
      prev_diff.normalize();
      next_diff.normalize();

      const double cross_prod_value = prev_diff[0] * next_diff[1] - prev_diff[1] * next_diff[0];
      
      double along_thickness;
      // Only in one case (we do not generate).
      if (cross_prod_value < -0.5) {
        along_thickness = 0.0;
      } else {
        along_thickness = outward_thickness;
      }
      
      Polygon_2 wall_polygon;
      CreateRectangle(line_room.walls[w], line_room.walls[next_w],
                      outward_thickness, inward_thickness, along_thickness, &wall_polygon);

      // Add doorwalls.
      if (doorwalls.find(make_pair(r, w)) != doorwalls.end()) {
        for (const auto& polygon : doorwalls[make_pair(r, w)]) {
          polygon_set.join(polygon);
        }
      }

      // Remove doorways.
      //Polygon_with_holes_2 output_polygon;

      /*
      std::list<Polygon_with_holes_2> lhs;
      {
        vector<Polygon_2> dummy;
        Polygon_with_holes_2 wall_polygon_tmp(wall_polygon, dummy.begin(), dummy.end());
        lhs.push_back(wall_polygon_tmp);
      }
      if (doorways.find(make_pair(r, w)) != doorways.end()) {
        for (const auto& hole : doorways[make_pair(r, w)]) {
          std::list<Polygon_with_holes_2> rhs;
          for (const auto& current : lhs) {
            CGAL::difference(current, hole, std::back_inserter(rhs));
          }
          lhs.swap(rhs);
        }
      }

      for (const auto& shape : lhs)
        polygon_set.join(shape);
      */
      polygon_set.join(wall_polygon);
    }
  }

  PolygonSetToFloorplanComponents(polygon_set, floorplan);
  SetFloorplanFloorHeight(line_floorplan, floorplan);
  // Set room interiors.  
  //  SetFloorplanRoomInteriors();

  
  //  FloorplanToSVG(
  /*
  list<Polygon_with_holes_2> res;
  list<Polygon_with_holes_2>::const_iterator it;
  cout << polygon_set.number_of_polygons_with_holes() << " components" << endl;
  polygon_set.polygons_with_holes(std::back_inserter(res));
  for (it = res.begin(); it != res.end(); ++it) {
    cout << "--> ";
    print_polygon_with_holes(*it);
  }
  */



  

  /*
  // Remove doors from the polygon.
  for (const auto& line_connection : line_floorplan.line_connections) {
    Vector2d diff = line_connection.first.positions[1] -
      line_connection.first.positions[0];
    diff.normalize();
    // ???? could be negative
    Vector2d ortho(diff[1], -diff[0]);
    ortho *= thickness;

    Polygon_2 polygon;
    polygon.push_back(Point_2(line_connection.first.positions[1][0] - ortho[0],
                              line_connection.first.positions[1][1] - ortho[1]));
    polygon.push_back(Point_2(line_connection.first.positions[0][0] - ortho[0],
                              line_connection.first.positions[0][1] - ortho[1]));
    polygon.push_back(Point_2(line_connection.second.positions[0][0] + ortho[0],
                              line_connection.second.positions[0][1] + ortho[1]));
    polygon.push_back(Point_2(line_connection.second.positions[1][0] + ortho[0],
                              line_connection.second.positions[1][1] + ortho[1]));

    //?????
    //    CGAL::difference(polygon_set.intersection(polygon);
  }
  */
  //----------------------------------------------------------------------
  // Set outer_boundaries, inner_boundaries.
  /*
      list<Polygon_with_holes_2> res;
      list<Polygon_with_holes_2>::const_iterator it;
      cout << S.number_of_polygons_with_holes() << " components" << endl;
      S.polygons_with_holes(std::back_inserter(res));
      for (it = res.begin(); it != res.end(); ++it) {
        cout << "--> ";
        print_polygon_with_holes(*it);
  */





  //----------------------------------------------------------------------
  // Set room_interiors.
}

void PolygonSetToFloorplanComponents(const Polygon_set_2& polygon_set, Floorplan* floorplan) {
  list<Polygon_with_holes_2> polygons;
  list<Polygon_with_holes_2>::const_iterator ite;
  // cout << polygon_set.number_of_polygons_with_holes() << " components" << endl;
  polygon_set.polygons_with_holes(std::back_inserter(polygons));
  for (ite = polygons.begin(); ite != polygons.end(); ++ite) {
    if (ite->is_unbounded()) {
      cerr << "Unbounded polygon..." << endl;
      exit (1);
    }

    FloorplanComponent component;
    {
      const Polygon_2& outer_boundary = ite->outer_boundary();
      for (Polygon_2::Vertex_const_iterator vertex = outer_boundary.vertices_begin();
           vertex != outer_boundary.vertices_end();
           ++vertex) {
        component.outer_shape.vertices.push_back(Vector2d(CGAL::to_double(vertex->x()),
                                                          CGAL::to_double(vertex->y())));
      }

      Cgal::Triangulate(component.outer_shape.vertices, &component.outer_shape.faces);      
    }
    {
      for (Polygon_with_holes_2::Hole_const_iterator hole_ite = ite->holes_begin();
           hole_ite != ite->holes_end();
           ++hole_ite) {
        const Polygon_2& hole = *hole_ite;
        Shape shape;
        for (Polygon_2::Vertex_const_iterator vertex = hole.vertices_begin();
             vertex != hole.vertices_end();
             ++vertex) {
          shape.vertices.push_back(Vector2d(CGAL::to_double(vertex->x()),
                                            CGAL::to_double(vertex->y())));
        }
        reverse(shape.vertices.begin(), shape.vertices.end());
        
        Cgal::Triangulate(shape.vertices, &shape.faces);
        
        component.inner_shapes.push_back(shape);        
      }
    }
    floorplan->components.push_back(component);
  }
}

void FloorplanToSVG(const Floorplan& floorplan, const string filename, const string filename2) {
  {
    ofstream ofstr;
    ofstr.open(filename.c_str());
    ofstr << "<svg>" << endl;
    for (const auto& component : floorplan.components) {
      ofstr << "<polyline points=\"";
      for (const auto& point : component.outer_shape.vertices) {
        ofstr << point[0] << ' ' << point[1] << ' ';
      }
      ofstr << component.outer_shape.vertices[0][0] << ' '
            << component.outer_shape.vertices[0][1] << ' ';
      //ofstr << "\" fill=\"rgb(255, 0, 255)\" stroke-width=\"3\" stroke=\"rgb(0,255,0)\"/>" << endl;
      ofstr << "\" stroke-width=\"1\" fill=\"none\" stroke=\"rgb(0,255,0)\"/>" << endl;    
      
      for (const auto& shape : component.inner_shapes) {
        ofstr << "<polyline points=\"";
        for (const auto& point : shape.vertices) {
          ofstr << point[0] << ' ' << point[1] << ' ';
        }
        ofstr << shape.vertices[0][0] << ' ' << shape.vertices[0][1] << ' ';
        //ofstr << "\" stroke-width=\"1\" fill=\"rgb(255, 255, 255)\" stroke=\"rgb(0,0,255)\"/>" << endl;
        ofstr << "\" stroke-width=\"1\" fill=\"none\" stroke=\"rgb(0,0,255)\"/>" << endl;
      }
    }
    ofstr << "</svg>" << endl;
    ofstr.close();
  }
  {
    ofstream ofstr;
    ofstr.open(filename2.c_str());
    ofstr << "<svg>" << endl;
    for (const auto& component : floorplan.components) {
      for (const auto& triangle : component.outer_shape.faces) {
        ofstr << "<polygon points=\"";
        for (int i = 0; i < 3; ++i) {
          ofstr << component.outer_shape.vertices[triangle[i]][0] << ','
                << component.outer_shape.vertices[triangle[i]][1] << ' ';
        }
        ofstr << "\" style=\"fill:lime;stroke:purple;stroke-width:1\" />" << endl;
      }
    }

    for (const auto& component : floorplan.components) {
      for (const auto& shape : component.inner_shapes) {
        for (const auto& triangle : shape.faces) {
          ofstr << "<polygon points=\"";
          for (int i = 0; i < 3; ++i) {
            ofstr << shape.vertices[triangle[i]][0] << ','
                  << shape.vertices[triangle[i]][1] << ' ';
          }
          ofstr << "\" style=\"fill:lightblue;stroke:purple;stroke-width:1\" />" << endl;
        }
      }
    }
    ofstr << "</svg>" << endl;
  }    
}

}  // namespace structured_indoor_modeling
  
