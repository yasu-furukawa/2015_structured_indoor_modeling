#include "../base/floorplan.h"
#include "../base/panorama.h"
#include "main_widget_util.h"
#include "panorama_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

bool IsInside(const Floorplan floorplan, const int room, const Vector2d& point) {
  const cv::Point2f point_tmp(point[0], point[1]);

  vector<cv::Point> contour;
  for (int w = 0; w < floorplan.GetNumWalls(room); ++w) {
    const Eigen::Vector2d local = floorplan.GetRoomVertexLocal(room, w);
    contour.push_back(cv::Point(local[0], local[1]));
  }

  if (cv::pointPolygonTest(contour, point_tmp, true) >= 0.0)
    return true;
  else
    return false;
}

}  // namespace

int FindPanoramaFromAirFloorplanClick(const std::vector<PanoramaRenderer>& panorama_renderers,
                                      const Eigen::Vector2d& pixel,
                                      const GLint viewport[],
                                      const GLdouble modelview[],
                                      const GLdouble projection[]) {
  int best_index = -1;
  double best_distance = 0.0;
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    GLdouble winX, winY, winZ;

    const Panorama& panorama = panorama_renderers[p].GetPanorama();
    gluProject(panorama.GetCenter()[0],
               panorama.GetCenter()[1],
               panorama.GetCenter()[2],
               modelview, projection, viewport,
               &winX, &winY, &winZ);

    const double distance = (pixel - Vector2d(winX, winY)).norm();
    if (best_index == -1 || distance < best_distance) {
      best_distance = distance;
      best_index = p;
    }
  }

  return best_index;
}

int FindRoomHighlighted(const Eigen::Vector2i& pixel,
                        const GLuint frameids[],
                        const GLint viewport[]) {
  unsigned char data;
  glBindFramebuffer(GL_FRAMEBUFFER, frameids[0]);
  glReadPixels(pixel[0], viewport[3] - pixel[1], 1, 1, GL_BLUE, GL_UNSIGNED_BYTE, &data);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  
  return static_cast<int>(data) - 1;
}

double ProgressFunction(const double elapsed,
                        const double offset,
                        const double fade_in_seconds,
                        const double fade_out_seconds) {
  return max(0.0,
             ((elapsed - offset) - fade_in_seconds) /
             (fade_out_seconds - fade_in_seconds));
}

double FadeFunction(const double elapsed,
                    const double offset,
                    const double fade_in_seconds,
                    const double fade_out_seconds) {
  // When a mouse keeps moving (offset is large, we draw with a full opacity).
  const double progress = ProgressFunction(elapsed, offset, fade_in_seconds, fade_out_seconds);
  const double kSpeed = 20.0;
  if (progress < 1.0 / kSpeed) {
    if (offset > fade_in_seconds)
      return 1.0;
    else
      return kSpeed * progress;
  } else if (progress > 0.75) {
    return 1.0 - (progress - 0.75) * 4.0;
  } else {
    return 1.0;
  }
}

double HeightAdjustmentFunction(const double elapsed,
                                const double offset,
                                const double fade_in_seconds,
                                const double fade_out_seconds) {
  // When a mouse keeps moving (offset is large, we draw with a full opacity).
  if (offset > fade_in_seconds)
    return 1.0;

  const double progress = ProgressFunction(elapsed, offset, fade_in_seconds, fade_out_seconds);
  return min(1.0, 0.2 + 10.0 * max(0.0, progress - 0.2));
}

void SetPanoramaToRoom(const Floorplan& floorplan,
                       const std::vector<PanoramaRenderer>& panorama_renderers,
                       std::map<int, int>* panorama_to_room) {
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();
  
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    const Vector3d global_center = panorama_renderers[p].GetPanorama().GetCenter();
    const Vector3d floorplan_center = floorplan_to_global.transpose() * global_center;
    const Vector2d floorplan_center2(floorplan_center[0], floorplan_center[1]);
           
    int room_id = -1;
    for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
      if (IsInside(floorplan, room, floorplan_center2)) {
        room_id = room;
        break;
      }
    }
    (*panorama_to_room)[p] = room_id;
  }
}

void SetRoomToPanorama(const Floorplan& floorplan,
                       const std::vector<PanoramaRenderer>& panorama_renderers,
                       std::map<int, int>* room_to_panorama) {
  const Eigen::Matrix3d& floorplan_to_global = floorplan.GetFloorplanToGlobal();

  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    const Vector2d room_center = floorplan.GetRoomCenterLocal(room);
    
    // Find the closest panorama.
    int best_panorama = -1;
    double best_distance = 0.0;
    for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
      const Vector3d global_center = panorama_renderers[p].GetPanorama().GetCenter();
      const Vector3d floorplan_center = floorplan_to_global.transpose() * global_center;
      const Vector2d panorama_center(floorplan_center[0], floorplan_center[1]);
      const double distance = (room_center - panorama_center).norm();
      if (best_panorama == -1 || distance < best_distance) {
        best_panorama = p;
        best_distance = distance;
      }
    }
    (*room_to_panorama)[room] = best_panorama;
  }
}

void SetPanoramaDistanceTable(const std::vector<PanoramaRenderer>& panorama_renderers,
                              std::vector<std::vector<double> >* panorama_distance_table) {
  panorama_distance_table->clear();
  panorama_distance_table->resize(panorama_renderers.size());
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    panorama_distance_table->at(p).resize(panorama_renderers.size(), -1);
  }
  for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
    for (int q = p + 1; q < (int)panorama_renderers.size(); ++q) {
      panorama_distance_table->at(p)[q] = panorama_distance_table->at(q)[p] =
        (ComputePanoramaDistance(panorama_renderers[p],
                                 panorama_renderers[q]) +
         ComputePanoramaDistance(panorama_renderers[q],
                                 panorama_renderers[p])) / 2.0;
    }
  }
}

double ComputePanoramaDistance(const PanoramaRenderer& lhs,
                               const PanoramaRenderer& rhs) {
  const Panorama& lhs_panorama = lhs.GetPanorama();
  const Panorama& rhs_panorama = rhs.GetPanorama();
  
  const Vector2d lhs_on_rhs_depth_image =
    rhs_panorama.RGBToDepth(rhs_panorama.Project(lhs_panorama.GetCenter()));
  // Search in some radius.
  const int wradius = rhs.DepthWidth() / 20;
  const int hradius = rhs.DepthHeight() / 20;
  const double distance = (lhs_panorama.GetCenter() - rhs_panorama.GetCenter()).norm();

  int connected = 0;
  int occluded  = 0;
  
  const int depth_width  = rhs.DepthWidth();
  const int depth_height = rhs.DepthHeight();

  const vector<Vector3d>& depth_mesh = rhs.DepthMesh();
  // Only look at the top half.
  for (int j = -hradius; j <= 0; ++j) {
    const int ytmp = static_cast<int>(round(lhs_on_rhs_depth_image[1])) + j;
    if (ytmp < 0 || depth_height <= ytmp)
      continue;
    for (int i = -wradius; i <= wradius; ++i) {
      const int xtmp = (static_cast<int>(round(lhs_on_rhs_depth_image[0])) + i + depth_width)
        % depth_width;
      const Vector3d depth_point = depth_mesh[ytmp * depth_width + xtmp];
      const double depthmap_distance = (depth_point - rhs_panorama.GetCenter()).norm();

      if (distance < depthmap_distance)
        ++connected;
      else
        ++occluded;
    }
  }
  if (connected + occluded == 0) {
    cerr << "Impossible in ComputePanoramaDistance" << endl;
    exit (1);
  }
  //cout << '(' << lhs << ',' << rhs << ',' << occluded *1.0/ (connected + occluded) << ") ";
  const double ratio = occluded / static_cast<double>(connected + occluded);
  const double kOffset = 0.0;
  const double kMinScale = 1.0;
  const double kMaxScale = 10.0;

  return distance *
    (kMinScale + (kMaxScale - kMinScale) * max(0.0, ratio - kOffset) /
     (1.0 - kOffset));
}

void FindPanoramaPath(const std::vector<PanoramaRenderer>& panorama_renderers,
                      const std::vector<std::vector<double> >& panorama_distance_table,
                      const int start_panorama,
                      const int goal_panorama,
                      std::vector<int>* indexes) {
  // Standard shortest path algorithm.
  const double kInvalid = -1.0;
  // Find connectivity information. Distance is put. -1 means not connected.
  vector<pair<double, int> > table(panorama_renderers.size(), pair<double, int>(kInvalid, -1));

  table[start_panorama] = pair<double, int>(0.0, start_panorama);
  // It is enough to repeat iterations at the number of panoramas - 1.
  for (int i = 0; i < (int)panorama_renderers.size() - 1; ++i) {
    // Update the result on panorama p by using q.
    for (int p = 0; p < (int)panorama_renderers.size(); ++p) {
      if (p == start_panorama)
        continue;
      for (int q = 0; q < (int)panorama_renderers.size(); ++q) {
        if (p == q)
          continue;
        // q is not reachable yet.
        if (table[q].first == kInvalid)
          continue;
        
        const double new_distance = table[q].first + panorama_distance_table[q][p];
        if (table[p].first == kInvalid || new_distance < table[p].first) {
          table[p].first = new_distance;
          table[p].second = q;
        }
      }
    }
  }

  if (table[goal_panorama].first == kInvalid) {
    cerr << "Impossible. every node is reachable." << endl;
    exit (1);
  }

  indexes->clear();
  int pindex = goal_panorama;
  indexes->push_back(pindex);
  while (pindex != start_panorama) {
    pindex = table[pindex].second;
    indexes->push_back(pindex);
  }
  reverse(indexes->begin(), indexes->end());

  cout << "Path ";
  for (int i = 0; i < (int)indexes->size(); ++i)
    cout << indexes->at(i) << ' ';
  cout << endl;
}
  
}  // namespace structured_indoor_modeling
  
