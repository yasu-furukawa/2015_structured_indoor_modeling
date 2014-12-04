#include <set>
#include "polygon_triangulation2.h"

using namespace Eigen;
using namespace std;

namespace {
bool IsConstraint(const int lhs, const int rhs, const int pnum) {
  const int diff = (lhs - rhs + pnum) % pnum;
  if (diff == 1 || diff == pnum - 1)
    return true;
  else
    return false;
}

}  // namespace

namespace Cgal {

typedef Delaunay2::Vertex_handle             Vertex2_handle;
typedef CDelaunay2::Vertex_handle           CVertex2_handle;
  
void Triangulate(const std::vector<Vector2d>& points,
                   std::vector<Vector3i>* triangles) {
  const int pnum = (int)points.size();

  CDelaunay2 cdel;
  std::map<CVertex2_handle, int> vertex_handle_to_index;
  vector<CVertex2_handle> vertex_handle;
  vertex_handle.resize(pnum);
  for (int i = 0; i < pnum; ++i) {
    vertex_handle[i] = cdel.insert(Point_2(points[i][0], points[i][1]));
    vertex_handle_to_index[vertex_handle[i]] = i;
  }
  for (int i = 0; i < pnum; ++i) {
    cdel.insert_constraint(vertex_handle[i],
                           vertex_handle[(i + 1) % pnum]);
  }
  
  CDelaunay2::Face_iterator fi = cdel.finite_faces_begin();
  while (fi != cdel.finite_faces_end()) {
    const int id0 = vertex_handle_to_index[fi->vertex(0)];
    const int id1 = vertex_handle_to_index[fi->vertex(1)];
    const int id2 = vertex_handle_to_index[fi->vertex(2)];

    triangles->push_back(Vector3i(id0, id1, id2));    
    fi++;
  }

  // A set of edges that means an outside/inside triangle.
  set<pair<int, int> > outside_edges, inside_edges;
  for (int i = 0; i < pnum; ++i) {
    outside_edges.insert(make_pair((i + 1) % pnum, i));
    inside_edges.insert(make_pair(i, (i + 1) % pnum));
  }  

  vector<Vector3i> inside_triangles;
  while (!triangles->empty()) {
    vector<Vector3i> remaining_triangles;
    for (const auto& triangle : (*triangles)) {
      if (outside_edges.find(make_pair(triangle[0], triangle[1])) != outside_edges.end() ||
          outside_edges.find(make_pair(triangle[1], triangle[2])) != outside_edges.end() ||
          outside_edges.find(make_pair(triangle[2], triangle[0])) != outside_edges.end()) {
        // Outside.
        for (int i = 0; i < 3; ++i) {
          if (!IsConstraint(triangle[(i + 1) % 3], triangle[i], pnum)) {
            outside_edges.insert(make_pair(triangle[(i + 1) % 3], triangle[i]));
          }
        }
      } else if (inside_edges.find(make_pair(triangle[0], triangle[1])) != inside_edges.end() ||
                 inside_edges.find(make_pair(triangle[1], triangle[2])) != inside_edges.end() ||
                 inside_edges.find(make_pair(triangle[2], triangle[0])) != inside_edges.end()) {
        // inside.
        for (int i = 0; i < 3; ++i) {
          if (!IsConstraint(triangle[(i + 1) % 3], triangle[i], pnum)) {
            inside_edges.insert(make_pair(triangle[(i + 1) % 3], triangle[i]));
          }
        }
        inside_triangles.push_back(triangle);
      } else {
        remaining_triangles.push_back(triangle);
      }
    }
    remaining_triangles.swap(*triangles);
  }

  inside_triangles.swap(*triangles);
}


/*
void Cdelaunay2::SetSub(void) {
  //----------------------------------------------------------------------
  for (int v = 0; v < (int)neighbors.size(); ++v) {
    vector<int> vtmp;    vtmp.swap(neighbors[v]);
    // in case of border, put the first vertex at the beginning of vtmp
    int firstv = 0;
    for (int i = 0; i < (int)vtmp.size(); i+=2) {
      const int id = vtmp[i];
      int safe = 1;
      for (int j = 0; j < (int)vtmp.size(); j+=2)
	if (vtmp[j+1] == id) {
	  safe = 0;	  break;
	}
      if (safe) {
	firstv = i;	break;
      }
    }

    // Since the cgal delaunay is randomized, we will make it deterministic
    // here. If not a border vertex, put the minimum id vertex in the
    // front.
    if (firstv == 0) {
      int minid = INT_MAX/2;
      for (int i = 0; i < (int)vtmp.size(); i+=2) {
        if (vtmp[i] < minid) {
          minid = vtmp[i];
          firstv = i;
        }
      }
    }
    
    if (firstv != 0) {
        const int itmp0 = vtmp[firstv];
        vtmp[firstv] = vtmp[0];
        vtmp[0] = itmp0;
        const int itmp1 = vtmp[firstv + 1];
        vtmp[firstv + 1] = vtmp[1];
        vtmp[1] = itmp1;
    }
    
    // Set neighbors here
    vector<int>::iterator index = vtmp.begin();    
    while (index != vtmp.end()) {
      neighbors[v].push_back(*index);   vtmp.erase(index);
      const int itmp = *index;            vtmp.erase(index);
          
      int pos = -1;
      for (int i = 0; i < (int)vtmp.size(); i+=2)
	if (vtmp[i] == itmp) {
	  pos = i;	  break;
	}
      // Keep on going
      if (pos != -1)
	index = vtmp.begin() + pos;
      // If not finding the next one.
      else {
        // Still something left
        if (!vtmp.empty()) {
          index = vtmp.begin();
          boundaries[v] = 2;
          neighbors[v].push_back(itmp);
        }
        // If finished but not closed
        else if (neighbors[v][0] != itmp) {
          if (boundaries[v] != 2)
            boundaries[v] = 1;
	  neighbors[v].push_back(itmp);
          index = vtmp.end();
	}
        // Fisnihed and closed
        else
          index = vtmp.end();          
      }
    }
  }
}

void Cdelaunay2::SetFneighbors(void) {
  fneighbors.clear();
  fneighbors.resize((int)triangles.size());

  // For each vertex, collect adjacent vertices
  vector<vector<int> > tids;
  tids.resize((int)neighbors.size());
  for (int t = 0; t < (int)triangles.size(); ++t)
    for (int i = 0; i < 3; ++i)
      tids[triangles[t][i]].push_back(t);
  
  for (int v = 0; v < (int)tids.size(); ++v) {
    for (int i = 0; i < (int)tids[v].size(); ++i) {
      const int tid0 = tids[v][i];
      Vec3i t0 = triangles[tid0];
      t0.sort();
      for (int j = i+1; j < (int)tids[v].size(); ++j) {
        const int tid1 = tids[v][j];
        Vec3i t1 = triangles[tid1];
        t1.sort();

        vector<int> common;
        // Check neighboring info
        if (t0[0] == t1[0] && t0[1] == t1[1] ||
            t0[0] == t1[1] && t0[1] == t1[2] ||
            t0[0] == t1[0] && t0[1] == t1[2] ||
            t0[1] == t1[0] && t0[2] == t1[1] ||
            t0[1] == t1[0] && t0[2] == t1[2] ||
            t0[1] == t1[1] && t0[2] == t1[2] ||
            t0[0] == t1[0] && t0[2] == t1[1] ||
            t0[0] == t1[0] && t0[2] == t1[2] ||
            t0[0] == t1[1] && t0[2] == t1[2]) {
          fneighbors[tid0].push_back(tid1);
          fneighbors[tid1].push_back(tid0);
        }
      }
    }
  }

  // unique
#pragma omp parallel for
  for (int f = 0; f < (int)fneighbors.size(); ++f) {
    if (fneighbors[f].empty())
      continue;
    
    sort(fneighbors[f].begin(), fneighbors[f].end());
    fneighbors[f].erase(unique(fneighbors[f].begin(),
                                 fneighbors[f].end()),
                          fneighbors[f].end());
  }
}
*/
}  // namespace Cgal
