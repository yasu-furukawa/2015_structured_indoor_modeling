#include <iostream>
#include "navigation.h"
#include "object_renderer.h"
#include "view_parameters.h"
#include "../base/detection.h"
#include "../base/file_io.h"
#include "../base/floorplan.h"
#include "../base/indoor_polygon.h"
#include "../base/point_cloud.h"

#ifdef __linux__
#include <GL/glu.h>
#elif _WIN32
#include <windows.h>
#include <GL/glu.h>
#else
#include <OpenGL/glu.h>
#endif

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

ObjectRenderer::ObjectRenderer(const Floorplan& floorplan,
                               const IndoorPolygon& indoor_polygon,
			       const Navigation& navigation,
                               const std::string& detection_file)
    : floorplan(floorplan), indoor_polygon(indoor_polygon), navigation(navigation) {
  render = true;

  ifstream ifstr;
  ifstr.open(detection_file.c_str());
  if (ifstr.is_open()) {
    ifstr >> detections;
    ifstr.close();
  }

  average_floor_height = 0.0;
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    average_floor_height += floorplan.GetFloorHeight(room);
  }
  average_floor_height /= max(1, floorplan.GetNumRooms());
}

ObjectRenderer::~ObjectRenderer() {
}

bool ObjectRenderer::Toggle() {
  render = !render;
  return render;
}

void ObjectRenderer::Precompute(const ViewParameters& view_parameters) {
  // vertices_middle.
  const double kMiddleProgress = 0.5;
  const double kBottomProgress = 1.0;
  const double kAnimation = 0.0;
  const Vector3d kNoOffset(0.0, 0.0, 0.0);

  Vector3d offset_direction;
  view_parameters.SetOffsetDirection(navigation.GetDirection(), &offset_direction);
  
  const Vector3d top_boundary = view_parameters.GetVerticalTopBoundary() * offset_direction;
  const Vector3d bottom_boundary = view_parameters.GetVerticalBottomBoundary() * offset_direction;
  
  for (int room = 0; room < (int)vertices.size(); ++room) {
    for (int object = 0; object < (int)vertices[room].size(); ++object) {
      for (int p = 0; p < (int)vertices[room][object].size(); p += 3) {
        const Vector3d global(vertices[room][object][p],
                              vertices[room][object][p + 1],
                              vertices[room][object][p + 2]);
        const Vector3d global_middle =
          view_parameters.TransformRoom(global, room, kMiddleProgress, kAnimation, kNoOffset);
        for (int i = 0; i < 3; ++i)
          vertices_middle[room][object][p + i] = global_middle[i];

        const Vector3d global_bottom =
          view_parameters.TransformObject(global, room, object, kBottomProgress, kAnimation, kNoOffset,
                                          top_boundary, bottom_boundary);
        for (int i = 0; i < 3; ++i)
          vertices_bottom[room][object][p + i] = global_bottom[i];
      }
    }
  }
}

void ObjectRenderer::Init(const string data_directory) {
  FileIO file_io(data_directory);

  vertices.clear();
  colors.clear();

  vertices.resize(floorplan.GetNumRooms());
  colors.resize(floorplan.GetNumRooms());
  centers.resize(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    PointCloud point_cloud;
    point_cloud.Init(file_io.GetRefinedObjectClouds(room));

    vertices[room].resize(point_cloud.GetNumObjects());
    colors[room].resize(point_cloud.GetNumObjects());
    centers[room].resize(point_cloud.GetNumObjects());

    cout << point_cloud.GetNumObjects() << " objects." << endl;

    vector<vector<int> >object_pt_num(point_cloud.GetNumObjects());
    
    for(int objid = 0; objid < (int)centers[room].size(); ++objid){
	centers[room][objid] = Vector3d(0,0,0);
	point_cloud.GetObjectIndice(objid, object_pt_num[objid]);
    }
    
    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Point& point = point_cloud.GetPoint(p);
      centers[room][point.object_id] += point.position;
      for (int i = 0; i < 3; ++i)
        vertices[room][point.object_id].push_back(point.position[i]);
      for (int i = 0; i < 3; ++i) {
        colors[room][point.object_id].push_back(point.color[i] / 255.0f);
      }
    }

    for(int objid=0; objid<point_cloud.GetNumObjects(); ++objid){
	if(object_pt_num[objid].size() != 0)
	    centers[room][objid] /= (double)object_pt_num[objid].size();
    }
  }

  vertices_org = vertices;
  colors_org = colors;

  vertices_middle = vertices;
  vertices_bottom = vertices;

  // ComputeBoundingBoxes2D();
}

void ObjectRenderer::InitGL() {
  initializeGLFunctions();
}
  
void ObjectRenderer::RenderAll(const double position) {
  if (!render)
    return;

  const bool kBlend = true; // ??? false

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  if (kBlend) {
    // glDisable(GL_DEPTH_TEST);
    glDepthMask(false);
    glEnable(GL_BLEND);
  }
  glEnable(GL_POINT_SMOOTH);

  const double kDurationPerObject = 0.4;
  
  for (int room = 0; room < (int)vertices.size(); ++room) {
      vector<pair<double, int> > distances_objects;
      for (int object_id = 0; object_id < (int)vertices[room].size(); ++object_id) {
	  // compute distance to the center of an object.
	  double distance_object = navigation.GetDirection().dot(centers[room][object_id]);
	  distances_objects.push_back(pair<double, int>(distance_object, object_id));
      }
      sort(distances_objects.rbegin(), distances_objects.rend());

    const double offset = (1.0 - kDurationPerObject) / max(1, (int)vertices[room].size() - 1);
    for (int object = 0; object < (int)vertices[room].size(); ++object) {
	const int object_id = distances_objects[object].second;
	// [object * offset, object * offset + kDurationPerObject].
	double scale;
      {
        const double start = object_id * offset;
        const double end = start + kDurationPerObject;
        if (position <= start || end <= position)
          scale = 1.0;
        else
          scale = sin(M_PI * (position - start) / kDurationPerObject) * 0.5 + 1.0;
      }
      
     for (int i = 0; i < (int)colors[room][object_id].size(); ++i) {
           colors[room][object_id][i] = min(1.0, scale * colors_org[room][object_id][i]);
     }
      
      glColorPointer(3, GL_FLOAT, 0, &colors[room][object_id][0]);
      glVertexPointer(3, GL_FLOAT, 0, &vertices[room][object_id][0]);
      
      if (kBlend) {
        glBlendColor(0, 0, 0, 0.5);
        //glBlendColor(0, 0, 0, 1.0);
        // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
        glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
      }
      glPointSize(0.5);
      
      glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object_id].size()) / 3);
    }
  }
  
	
  glDisable(GL_POINT_SMOOTH);
  if (kBlend) {
    glDisable(GL_BLEND);
    glDepthMask(true);
    // glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void ObjectRenderer::RenderAll(const ViewParameters& view_parameters,
                               const double air_to_tree_progress,
                               const double animation,
                               const Eigen::Vector3d& offset_direction) {
  if (!render)
    return;

  const bool kBlend = true;
  const Vector3d kNoOffset(0.0, 0.0, 0.0);
  if (air_to_tree_progress < 1.0) {
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    if (kBlend) {
      // glDisable(GL_DEPTH_TEST);
      glDepthMask(false);
      glEnable(GL_BLEND);
    }
    glEnable(GL_POINT_SMOOTH);
    
    // const double kDurationPerObject = 0.4;
    vector<float> positions;  
    for (int room = 0; room < (int)vertices.size(); ++room) {
      for (int row = ViewParameters::kMaxNumRows - 1; row >= 0; --row) {
        for (int object = 0; object < (int)vertices[room].size(); ++object) {
          if (view_parameters.GetObjectRow(room, object) != row)
            continue;
          
          glColorPointer(3, GL_FLOAT, 0, &colors_org[room][object][0]);
          
          positions.resize(vertices[room][object].size());
          
          if (air_to_tree_progress < 0.5) {
            for (int p = 0; p < (int)vertices[room][object].size(); ++p)
              positions[p] = 2.0 * (0.5 - air_to_tree_progress) * vertices[room][object][p] +
                2.0 * air_to_tree_progress * vertices_middle[room][object][p];
          } else {
            for (int p = 0; p < (int)vertices[room][object].size(); ++p)
              positions[p] = 2.0 * (1.0 - air_to_tree_progress) * vertices_middle[room][object][p] +
                2.0 * (air_to_tree_progress - 0.5) * vertices_bottom[room][object][p];
          }
          
          glVertexPointer(3, GL_FLOAT, 0, &positions[0]);
          
          if (kBlend) {
            glBlendColor(0, 0, 0, 0.5);
            //glBlendColor(0, 0, 0, 1.0);
            // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
            glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
          }
          glPointSize(0.5);
          
          glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object].size()) / 3);
        }
      }
    }
  } else {
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    if (kBlend) {
      // glDisable(GL_DEPTH_TEST);
      glDepthMask(false);
      glEnable(GL_BLEND);
    }
    glEnable(GL_POINT_SMOOTH);
    
    // const double kDurationPerObject = 0.4;
    for (int room = 0; room < (int)vertices.size(); ++room) {
      for (int row = ViewParameters::kMaxNumRows - 1; row >= 0; --row) {
        for (int object = 0; object < (int)vertices[room].size(); ++object) {
          if (view_parameters.GetObjectRow(room, object) != row)
            continue;
          
          glColorPointer(3, GL_FLOAT, 0, &colors_org[room][object][0]);

          double global_to_local[16], local_to_global[16];
          for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 4; ++x) {
              const int index = x * 4 + y;
              if (x == y)
                global_to_local[index] = local_to_global[index] = 1.0;
              else
                global_to_local[index] = local_to_global[index] = 0.0;
            }
          }
          global_to_local[0] = view_parameters.x_axis[0];
          global_to_local[4] = view_parameters.x_axis[1];
          global_to_local[8] = view_parameters.x_axis[2];
          global_to_local[1] = view_parameters.y_axis[0];
          global_to_local[5] = view_parameters.y_axis[1];
          global_to_local[9] = view_parameters.y_axis[2];
          global_to_local[2] = view_parameters.z_axis[0];
          global_to_local[6] = view_parameters.z_axis[1];
          global_to_local[10] = view_parameters.z_axis[2];

          local_to_global[0] = view_parameters.x_axis[0];
          local_to_global[1] = view_parameters.x_axis[1];
          local_to_global[2] = view_parameters.x_axis[2];
          local_to_global[4] = view_parameters.y_axis[0];
          local_to_global[5] = view_parameters.y_axis[1];
          local_to_global[6] = view_parameters.y_axis[2];
          local_to_global[8] = view_parameters.z_axis[0];
          local_to_global[9] = view_parameters.z_axis[1];
          local_to_global[10] = view_parameters.z_axis[2];

          double angle;
          if (animation < 0.25)
            angle = 360.0 * animation;
          else if (0.25 <= animation && animation < 0.75)
            angle = 360.0 * (0.5 - animation);
          else
            angle = 360.0 * animation;
          
          const Vector3d vertical_object_top = view_parameters.GetVerticalTopBoundary() * offset_direction;
          const Vector3d vertical_object_bottom = view_parameters.GetVerticalBottomBoundary() * offset_direction;
          const Vector3d row_shift = (vertical_object_bottom - vertical_object_top) / ViewParameters::kMaxNumRows;
          
          const double scale = view_parameters.object_configurations[room][object].scale;
          const Vector3d global_displacement =
            (view_parameters.LocalToGlobalNormal(view_parameters.room_configurations[room].displacement +
                                                 view_parameters.object_configurations[room][object].displacement) +
             vertical_object_top + (0.5 + view_parameters.object_configurations[room][object].row) * row_shift);
          
          const Vector3d global_center =
            view_parameters.LocalToGlobal(view_parameters.object_configurations[room][object].center);
          
          glMatrixMode(GL_MODELVIEW);
          glPushMatrix();
          glTranslated(global_displacement[0], global_displacement[1], global_displacement[2]);
          
          glTranslated(global_center[0], global_center[1], global_center[2]);
          glMultMatrixd(local_to_global);

          glRotated(angle, 0, 0, 1);
          glScaled(scale, scale, scale);
          glMultMatrixd(global_to_local);
          glTranslated(- global_center[0], - global_center[1], - global_center[2]);

          glVertexPointer(3, GL_FLOAT, 0, &vertices[room][object][0]);
          
          if (kBlend) {
            glBlendColor(0, 0, 0, 0.5);
            //glBlendColor(0, 0, 0, 1.0);
            // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
            glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
          }
          glPointSize(0.5);
          
          glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object].size()) / 3);

          glPopMatrix();
        }
      }
    }
  }
      
	
  glDisable(GL_POINT_SMOOTH);
  if (kBlend) {
    glDisable(GL_BLEND);
    glDepthMask(true);
    // glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  
  /*
    // Slow
  if (!render)
    return;
  
  const bool kBlend = true; // ??? false
  const Vector3d kNoOffset(0.0, 0.0, 0.0);

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  if (kBlend) {
    // glDisable(GL_DEPTH_TEST);
    glDepthMask(false);
    glEnable(GL_BLEND);
  }
  glEnable(GL_POINT_SMOOTH);

  // const double kDurationPerObject = 0.4;
  vector<float> positions;  
  for (int room = 0; room < (int)vertices.size(); ++room) {
    for (int row = ViewParameters::kMaxNumRows - 1; row >= 0; --row) {
      for (int object = 0; object < (int)vertices[room].size(); ++object) {
        if (view_parameters.GetObjectRow(room, object) != row)
          continue;

        glColorPointer(3, GL_FLOAT, 0, &colors_org[room][object][0]);

        positions = vertices[room][object];
        for (int p = 0; p < (int)positions.size(); p += 3) {
          Vector3d position(positions[p], positions[p + 1], positions[p + 2]);
          // Adjust with respect to the object center.
          position = view_parameters.TransformObject(position, room, object, air_to_tree_progress,
                                                     animation, kNoOffset,
                                                     view_parameters.GetVerticalTopBoundary() * offset_direction,
                                                     view_parameters.GetVerticalBottomBoundary() * offset_direction);
          
          for (int a = 0; a < 3; ++a)
            positions[p + a] = position[a];
        }
        glVertexPointer(3, GL_FLOAT, 0, &positions[0]);
        
        if (kBlend) {
          glBlendColor(0, 0, 0, 0.5);
          //glBlendColor(0, 0, 0, 1.0);
          // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
          glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
        }
        glPointSize(1.0);
        
        glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object].size()) / 3);
      }
    }
  }
  
	
  glDisable(GL_POINT_SMOOTH);
  if (kBlend) {
    glDisable(GL_BLEND);
    glDepthMask(true);
    // glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  */

  





  /*
  if (!render)
    return;

  const double shrink_ratio = air_to_tree_progress * max_shrink_ratio + (1.0 - air_to_tree_progress);
  const Vector3d vertical_shift(0, 0, air_to_tree_progress * max_vertical_shift);
  Matrix3d rotation;
  rotation(0, 0) = cos(animation * 2 * M_PI);
  rotation(0, 1) = -sin(animation * 2 * M_PI);
  rotation(0, 2) = 0.0;
  rotation(1, 0) = sin(animation * 2 * M_PI);
  rotation(1, 1) = cos(animation * 2 * M_PI);
  rotation(1, 2) = 0.0;
  rotation(2, 0) = 0.0;
  rotation(2, 1) = 0.0;
  rotation(2, 2) = 1.0;

  
  const bool kBlend = true; // ??? false

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  if (kBlend) {
    // glDisable(GL_DEPTH_TEST);
    glDepthMask(false);
    glEnable(GL_BLEND);
  }
  glEnable(GL_POINT_SMOOTH);

  const double kDurationPerObject = 0.4;

  vector<float> positions;  
  for (int room = 0; room < (int)vertices.size(); ++room) {
    const BoundingBox& bounding_box = view_parameters.GetFloorplanDeformation().room_bounding_boxes[room];
    const Vector3d room_center = (bounding_box.min_xyz + bounding_box.max_xyz) / 2.0;

    for (int object = 0; object < (int)vertices[room].size(); ++object) {
      glColorPointer(3, GL_FLOAT, 0, &colors_org[room][object][0]);

      positions = vertices[room][object];
      for (int p = 0; p < (int)positions.size(); p += 3) {
        Vector3d position(positions[p], positions[p + 1], positions[p + 2]);
        // Adjust with respect to the object center.
        position = room_center + rotation * (position - room_center) * shrink_ratio;
        position += vertical_shift;
        for (int a = 0; a < 3; ++a)
          positions[p + a] = position[a];
      }
      glVertexPointer(3, GL_FLOAT, 0, &positions[0]);

      if (kBlend) {
        glBlendColor(0, 0, 0, 0.5);
        //glBlendColor(0, 0, 0, 1.0);
        // glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
        glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA);
      }
      glPointSize(1.0);
      
      glDrawArrays(GL_POINTS, 0, ((int)vertices[room][object].size()) / 3);
    }
  }
  
	
  glDisable(GL_POINT_SMOOTH);
  if (kBlend) {
    glDisable(GL_BLEND);
    glDepthMask(true);
    // glEnable(GL_DEPTH_TEST);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  */
}
  
void ObjectRenderer::ComputeBoundingBoxes2D() {
  const int num_rooms = vertices.size();
  bounding_boxes_2D.resize(num_rooms);
  for (int room = 0; room < num_rooms; ++room) {
    const int num_objects = vertices[room].size();
    bounding_boxes_2D[room].resize(num_objects);
    for (int object = 0; object < num_objects; ++object) {

      Vector3d min_xyz, max_xyz;
      for (int p = 0; p < (int)vertices[room][object].size(); p += 3) {
        const Vector3d point(vertices[room][object][p + 0],
                             vertices[room][object][p + 1],
                             vertices[room][object][p + 2]);
        
        const Vector3d manhattan = indoor_polygon.GlobalToManhattan(point);
        if (p == 0) {
          min_xyz = max_xyz = manhattan;
        } else {
          for (int a = 0; a < 3; ++a) {
            min_xyz[a] = min(min_xyz[a], manhattan[a]);
            max_xyz[a] = max(max_xyz[a], manhattan[a]);
          }
        }
      }

      const double average_z = (min_xyz[2] + max_xyz[2]) / 2.0;
      BoundingBox2D& bounding_box_2D = bounding_boxes_2D[room][object];
      bounding_box_2D.corners[0] =
        indoor_polygon.ManhattanToGlobal(Vector3d(min_xyz[0], min_xyz[1], average_z));
      bounding_box_2D.corners[1] =
        indoor_polygon.ManhattanToGlobal(Vector3d(max_xyz[0], min_xyz[1], average_z));
      bounding_box_2D.corners[2] =
        indoor_polygon.ManhattanToGlobal(Vector3d(max_xyz[0], max_xyz[1], average_z));
      bounding_box_2D.corners[3] =
        indoor_polygon.ManhattanToGlobal(Vector3d(min_xyz[0], max_xyz[1], average_z));
    }
  }
}

void ObjectRenderer::RenderDesk(const Detection& /* detection */,
                                const Eigen::Vector3d vs[4],
                                const double animation) const {
  const double kAnimationRatio = 0.1;

  const Vector3d center = (vs[0] + vs[1] + vs[2] + vs[3]) / 4.0;
  Vector3d new_vs[4];
  for (int i = 0; i < 4; ++i)
    new_vs[i] = vs[i] + (vs[i] - center) * animation * kAnimationRatio;

  const double kCornerRatio = 0.1;
  
  vector<Vector3d> points;
  
  for (int current = 0; current < 4; ++current) {
    const int prev = (current - 1 + 4) % 4;
    const int next = (current + 1) % 4;

    Vector3d prev_diff = (new_vs[prev] - new_vs[current]) * kCornerRatio;
    Vector3d next_diff = (new_vs[next] - new_vs[current]) * kCornerRatio;

    const double prev_length = prev_diff.norm();
    const double next_length = next_diff.norm();
    const double min_length = min(prev_length, next_length);
    prev_diff *= min_length / prev_length;
    next_diff *= min_length / next_length;    

    const Vector3d internal = new_vs[current] + prev_diff + next_diff;
    
    // First line.
    points.push_back((new_vs[prev] + new_vs[current]) / 2.0);
    points.push_back(new_vs[current] + prev_diff);
    // Corner.
    const int kNumSamples = 6;
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = M_PI / 2.0 * (s + 0.5) / kNumSamples;
      points.push_back(internal - cos(angle) * next_diff - sin(angle) * prev_diff);
    }
    // Second line.
    points.push_back(new_vs[current] + next_diff);
  }
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(0.8f, 1.0f, 0.8f, 0.5f + 0.5f * animation);
  for (const auto& point : points) {
    glVertex3d(point[0], point[1], point[2]);
  }
  glEnd();

  glLineWidth(1.5f);
  glBegin(GL_LINE_LOOP);
  glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
  for (const auto& point : points) {
    glVertex3d(point[0], point[1], point[2]);
  }
  glEnd();

  glDisable(GL_BLEND);
}

void ObjectRenderer::RenderSofa(const Detection& /* detection */,
                                const Eigen::Vector3d vs[4],
                                const double animation) const {
  const Vector3d x_diff = vs[1] - vs[0];
  const Vector3d y_diff = vs[3] - vs[0];
  vector<vector<Vector2d> > points_for_lines, points_for_polygons;

  const int num_cushions = static_cast<int>(round(x_diff.norm() / y_diff.norm()));

  {
    vector<Vector2d> outer_frame;
    outer_frame.push_back(Vector2d(0, 0.05));
    outer_frame.push_back(Vector2d(0, 1.0));
    outer_frame.push_back(Vector2d(1.0, 1.0));
    outer_frame.push_back(Vector2d(1.0, 0.05));
    outer_frame.push_back(Vector2d(0.875, 0.05));
    outer_frame.push_back(Vector2d(0.875, 0.75));
    outer_frame.push_back(Vector2d(0.125, 0.75));
    outer_frame.push_back(Vector2d(0.125, 0.05));

    points_for_lines.push_back(outer_frame);
  }
  for (int c = 0; c < num_cushions; ++c) {
    vector<Vector2d> cushion;
    const double min_x = 0.125 + c * 0.75 / num_cushions;
    const double max_x = 0.125 + (c + 1) * 0.75 / num_cushions;
    const double min_y = - 0.3 * animation;
    const double max_y = 0.75;
    
    cushion.push_back(Vector2d(min_x, min_y));
    cushion.push_back(Vector2d(min_x, max_y));
    cushion.push_back(Vector2d(max_x, max_y));
    cushion.push_back(Vector2d(max_x, min_y));

    points_for_lines.push_back(cushion);
  }

  //----------------------------------------------------------------------
  {
    vector<Vector2d> outer_frame;
    outer_frame.push_back(Vector2d(0.125, 0.05));
    outer_frame.push_back(Vector2d(0.125, 0.75));
    outer_frame.push_back(Vector2d(0, 1.0));
    outer_frame.push_back(Vector2d(0, 0.05));

    points_for_polygons.push_back(outer_frame);
  }
  {
    vector<Vector2d> outer_frame;
    outer_frame.push_back(Vector2d(0.875, 0.75));
    outer_frame.push_back(Vector2d(1.0, 1.0));
    outer_frame.push_back(Vector2d(0, 1.0));
    outer_frame.push_back(Vector2d(0.125, 0.75));

    points_for_polygons.push_back(outer_frame);
  }
  {
    vector<Vector2d> outer_frame;
    outer_frame.push_back(Vector2d(0.875, 0.75));
    outer_frame.push_back(Vector2d(0.875, 0.05));
    outer_frame.push_back(Vector2d(1.0, 0.05));
    outer_frame.push_back(Vector2d(1.0, 1.0));

    points_for_polygons.push_back(outer_frame);
  }
  for (int c = 0; c < num_cushions; ++c) {
    vector<Vector2d> cushion;
    const double min_x = 0.125 + c * 0.75 / num_cushions;
    const double max_x = 0.125 + (c + 1) * 0.75 / num_cushions;
    const double min_y = - 0.3 * animation;
    const double max_y = 0.75;
    
    cushion.push_back(Vector2d(max_x, min_y));
    cushion.push_back(Vector2d(max_x, max_y));
    cushion.push_back(Vector2d(min_x, max_y));
    cushion.push_back(Vector2d(min_x, min_y));

    points_for_polygons.push_back(cushion);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (int p = 0; p < (int)points_for_polygons.size(); ++p) {
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(1.0f, 0.8f, 0.8f, 0.5f + 0.5f * animation);
    for (const auto& uv : points_for_polygons[p]) {
      const Vector3d point = vs[0] + uv[0] * x_diff + uv[1] * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }
  
  glLineWidth(1.5f);
  for (int p = 0; p < (int)points_for_lines.size(); ++p) {
    glBegin(GL_LINE_LOOP);
    glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
    for (const auto& uv : points_for_lines[p]) {
      const Vector3d point = vs[0] + uv[0] * x_diff + uv[1] * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }
}

void ObjectRenderer::RenderLamp(const Detection& /* detection */,
                                const Eigen::Vector3d vs[4],
                                const double animation) const {
  const double kLargeRadius = 1.0 + 0.2 * animation;
  const double kSmallRadius = 0.4 + 0.2 * animation;
  const double kTinyRadius = 0.2 + 0.2 * animation;

  const float kFillAlpha = 0.5f + 0.5f * animation;
  const float kLineAlpha = 1.0f;
  // Outer frame.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(1.0f, 1.0f, 1.0f, kFillAlpha);
  for (int i = 0; i < 4; ++i)
    glVertex3d(vs[i][0], vs[i][1], vs[i][2]);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glColor4f(0.0f, 0.0f, 0.0f, kLineAlpha);
  for (int i = 0; i < 4; ++i)
    glVertex3d(vs[i][0], vs[i][1], vs[i][2]);
  glEnd();

  const Vector3d center = (vs[0] + vs[1] + vs[2] + vs[3]) / 4.0;
  Vector3d x_diff = (vs[1] - vs[0]) / 2.0;
  Vector3d y_diff = (vs[3] - vs[0]) / 2.0;

  const double x_length = x_diff.norm();
  const double y_length = y_diff.norm();
  const double length = min(x_length, y_length);
  x_diff *= length / x_length;
  y_diff *= length / y_length;
  
  {
    const int kNumSamples = 20;
    glBegin(GL_TRIANGLE_FAN);
    // Yellow.
    glColor4f(1.0f, 1.0f, 0.0f, kFillAlpha);
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = 2.0 * M_PI * s / kNumSamples;
      const Vector3d point = center + kLargeRadius * cos(angle) * x_diff + kLargeRadius * sin(angle) * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);

    glColor4f(0.0f, 0.0f, 0.0f, kLineAlpha);
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = 2.0 * M_PI * s / kNumSamples;
      const Vector3d point = center + kLargeRadius * cos(angle) * x_diff + kLargeRadius * sin(angle) * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }

  {
    const int kNumSamples = 20;
    glBegin(GL_TRIANGLE_FAN);
    // White opaque.
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = 2.0 * M_PI * s / kNumSamples;
      const Vector3d point = center + kSmallRadius * cos(angle) * x_diff + kSmallRadius * sin(angle) * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor4f(0.0f, 0.0f, 0.0f, kLineAlpha);
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = 2.0 * M_PI * s / kNumSamples;
      const Vector3d point = center + kSmallRadius * cos(angle) * x_diff + kSmallRadius * sin(angle) * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }

  {
    const int kNumSamples = 20;
    glBegin(GL_TRIANGLE_FAN);
    // White opaque.
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    for (int s = 0; s < kNumSamples; ++s) {
      const double angle = 2.0 * M_PI * s / kNumSamples;
      const Vector3d point = center + kTinyRadius * cos(angle) * x_diff + kTinyRadius * sin(angle) * y_diff;
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }

  /*
  {
    const double angles[3] = { 0.0, M_PI * 2 / 3, M_PI * 4 / 3 };
    
    glBegin(GL_LINES);
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

    for (int i = 0; i < 3; ++i) {
      glVertex3d(center[0], center[1], center[2]);
      const Vector3d point = center + kSmallRadius * cos(angles[i]) * x_diff + kSmallRadius * sin(angles[i]) * y_diff;
      
      glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
  }
  */
  
  glDisable(GL_BLEND);
}

void ObjectRenderer::RenderName(const Detection& detection,
                                const Eigen::Vector3d vs[4],
                                const double animation,
                                const GLint viewport[],
                                const GLdouble modelview[],
                                const GLdouble projection[],                                
                                QGLWidget* widget) const {
  if (animation < 0.5)
    return;

  if (detection.names.size() == 0)
    return;

  if (detection.names[0] == "unknown")
    return;
  
  string full_name("");
  for (const auto& word : detection.names) {
    full_name = full_name + string(" ") + word;
  }

  const Vector3d center = (vs[0] + vs[1] + vs[2] + vs[3]) / 4.0;
  
  GLdouble u, v, w;
  gluProject(center[0], center[1], center[2], modelview, projection, viewport, &u, &v, &w);
  
  const QFont font("Times", 14);
  const double kCharacterWidth = 7.25;
  widget->renderText(u - full_name.length() * kCharacterWidth / 2.0,
                     viewport[3] - v,
                     full_name.c_str(),
                     font);
  }

  
void ObjectRenderer::RenderDefault(const Detection& /* detection */,
                                   const Eigen::Vector3d /* vs */ [4],
                                   const double /* animation */) const {
  

}

void ObjectRenderer::RotateToFrontal(const Detection& detection,
                                     const Eigen::Vector3d vs[4],
                                     Eigen::Vector3d rotated_vs[4]) const {
  const Vector3d center = (vs[0] + vs[1] + vs[2] + vs[3]) / 4.0;
  // Identify the longest side.
  const Vector3d x_diff = vs[1] - vs[0];
  const Vector3d y_diff = vs[3] - vs[0];
  const double width  = x_diff.norm();
  const double height = y_diff.norm();

  // Identify the frontal direction.
  const Vector3d room_center = floorplan.GetRoomCenterGlobal(detection.room);
  const Vector3d diff = room_center - center;

  // Identify frontal side and shift vs.
  int shift;
  if (width > height) {
    if (y_diff.dot(diff) < 0.0)
      shift = 0;
    else
      shift = 2;
  } else {
    if (x_diff.dot(diff) < 0.0)
      shift = 1;
    else
      shift = 3;
  }

  for (int i = 0; i < 4; ++i)
    rotated_vs[(i + shift) % 4] = vs[i];
}
  
void ObjectRenderer::RenderIcons(const double /* alpha */,
                                 const double animation,
                                 const GLint viewport[],
                                 const GLdouble modelview[],
                                 const GLdouble projection[],
                                 QGLWidget* widget) {
  // Depending on detection.name, change.
  for (const auto& detection : detections) {
    if (detection.room == -1)
      continue;
    
    // const double z = (detection.ranges[2][0] + detection.ranges[2][1]) / 2.0;
    
    Vector3d vs[4] = { Vector3d(detection.ranges[0][0], detection.ranges[1][0], average_floor_height),
                       Vector3d(detection.ranges[0][1], detection.ranges[1][0], average_floor_height),
                       Vector3d(detection.ranges[0][1], detection.ranges[1][1], average_floor_height),
                       Vector3d(detection.ranges[0][0], detection.ranges[1][1], average_floor_height) };
    for (int i = 0; i < 4; ++i)
      vs[i] = indoor_polygon.ManhattanToGlobal(vs[i]);

    Vector3d rotated_vs[4];
    RotateToFrontal(detection, vs, rotated_vs);
    
    if (detection.names[0] == "table") {
      RenderDesk(detection, rotated_vs, animation);
    } else if (detection.names[0] == "sofa") {
      RenderSofa(detection, rotated_vs, animation);
    } else if (detection.names[0] == "lamp") {
      RenderLamp(detection, rotated_vs, animation);
    } else {
      // RenderDefault(detection, rotated_vs);
      RenderDesk(detection, rotated_vs, animation);
    }

    RenderName(detection, rotated_vs, animation, viewport, modelview, projection, widget);
  }
  
  /*
  glEnable(GL_BLEND);
  
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glColor4f(0.8f, 1.0f, 0.8f, 0.5f);
  for (int room = 0; room < (int)bounding_boxes.size(); ++room) {
    for (const auto& bounding_box : bounding_boxes[room]) {
      for (int i = 0; i < 4; ++i)
        glVertex3d(bounding_box.corners[i][0],
                   bounding_box.corners[i][1],
                   bounding_box.corners[i][2]);
    }
  }
  glEnd();

  glLineWidth(0.5f);
  for (int room = 0; room < (int)bounding_boxes.size(); ++room) {
    for (const auto& bounding_box : bounding_boxes[room]) {
      glBegin(GL_LINE_LOOP);
      glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
      for (int i = 0; i < 4; ++i)
        glVertex3d(bounding_box.corners[i][0],
                   bounding_box.corners[i][1],
                   bounding_box.corners[i][2]);
      glEnd();
    }
  }
  
  glDisable(GL_BLEND);
  */
}

int ObjectRenderer::GetNumRooms() const {
  return vertices.size();
}
 
int ObjectRenderer::GetNumObjects(const int room) const {
  return vertices[room].size();
}

const std::vector<float>& ObjectRenderer::GetObject(const int room, const int object) const {
  return vertices[room][object];
}
  
}  // namespace structured_indoor_modeling
  
