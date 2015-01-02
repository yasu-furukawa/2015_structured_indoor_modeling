#ifndef FLOORPLAN_UTIL_H__
#define FLOORPLAN_UTIL_H__

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/General_polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>

#include "../base/floorplan.h"

namespace structured_indoor_modeling {

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef CGAL::Polygon_set_2<Kernel>                       Polygon_set_2;
// typedef CGAL::General_polygon_with_holes_2<Kernel>         General_polygon_with_holes_2;
// typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;


// Conversion from the input to the output.
void ConvertLineFloorplanToFloorplan(const LineFloorplan& line_floorplan,
                                     const double outward_thickness,
                                     const double inward_thickness,
                                     Floorplan* floorplan);

void PolygonSetToFloorplanComponents(const Polygon_set_2& polygon_set, Floorplan* floorplan);

void FloorplanToSVG(const Floorplan& floorplan,
                    const std::string filename,
                    const std::string filename2);

}  // namespace structured_indoor_modeling
 
#endif  // FLOORPLAN_UTIL_H__
