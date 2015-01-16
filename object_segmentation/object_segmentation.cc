#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>

#include "../base/floorplan.h"
#include "../base/point_cloud.h"
#include "../base/kdtree/KDtree.h"
#include "object_segmentation.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

bool IsOnFloor(const double floor_height, const double margin, const Point& point) {
  return point.position[2] - floor_height <= margin;
}
  
bool IsOnWall(const std::vector<cv::Point>& contour,
              const double wall_margin,
              const Point& point) {
  const cv::Point local(point.position[0], point.position[1]);
  return fabs(cv::pointPolygonTest(contour, local, true)) <= wall_margin;
}

bool IsOnCeiling(const double ceiling_height, const double margin, const Point& point) {
  return ceiling_height - point.position[2] <= margin;
}  

double PointDistance(const Point& lhs, const Point& rhs) {
  const Vector3d diff = lhs.position - rhs.position;
  return (fabs(diff.dot(lhs.normal)) + fabs(diff.dot(rhs.normal))) / 2.0;
}

void AssignSamples(const std::vector<Point>& points,
                   const std::vector<
                   std::vector<int>* segments)
  
}  // namespace

void SetRoomOccupancy(const Floorplan& floorplan,
                      std::vector<int>* room_occupancy) {
  const Vector3i grid_size = floorplan.GetGridSize();
  room_occupancy->clear();
  const int kBackground = -1;
  room_occupancy->resize(grid_size[0] * grid_size[1], kBackground);

  // Make room contour polygons for each room.
  vector<vector<cv::Point> > room_contours(floorplan.GetNumRooms());
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
      const Vector2d local = floorplan.GetRoomVertexLocal(room, vertex);
      room_contours[room].push_back(cv::Point(local[0], local[1]));
    }
  }

  const double kDistanceThreshold = floorplan.GetGridUnit() * 2.0;
  int index = 0;
  for (int y = 0; y < grid_size[1]; ++y) {
    for (int x = 0; x < grid_size[0]; ++x, ++index) {
      const Vector2d local = floorplan.GridToLocal(Vector2d(x, y));
      const cv::Point point(local[0], local[1]);

      // Find the room with the closest distance
      const int kInvalid = -1;
      int best_room = kInvalid;
      double best_distance = 0.0;

      for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
        const double distance = cv::pointPolygonTest(room_contours[room], point, true);
        if (best_room == kInvalid || distance > best_distance) {
          best_room = room;
          best_distance = distance;

          // Inside.
          if (best_distance > 0.0)
            break;
        }
      }
      
      if (best_distance > - kDistanceThreshold) {
        room_occupancy->at(index) = best_room;
      }
    }
  }
}

void CollectPointsInRoom(const std::vector<PointCloud>& point_clouds,
                         const Floorplan& floorplan,
                         const std::vector<int>& room_occupancy,
                         const int room,
                         std::vector<Point>* points) {
  for (const auto& point_cloud : point_clouds) {
    for (int p = 0; p < point_cloud.GetNumPoints(); ++p) {
      const Vector3d& local = point_cloud.GetPoint(p).position;
      const Vector2i grid_int = floorplan.LocalToGridInt(Vector2d(local[0], local[1]));
      const int index = grid_int[1] * floorplan.GetGridSize()[0] + grid_int[0];
      if (room_occupancy[index] == room)
        points->push_back(point_cloud.GetPoint(p));
    }
  }
}

void IdentifyFloorWallCeiling(const std::vector<Point>& points,
                              const Floorplan& floorplan,
                              const std::vector<int>& room_occupancy,
                              const int room,
                              std::vector<int>* segments) {
  const double kFloorMarginRatio   = 0.1;
  const double kCeilingMarginRatio = 0.1;
  const double kWallMarginRatio    = 0.03;
  const double room_height    = floorplan.GetCeilingHeight(room) - floorplan.GetFloorHeight(room);
  const double floor_margin   = room_height * kFloorMarginRatio;
  const double wall_margin    = room_height * kWallMarginRatio;
  const double ceiling_margin = room_height * kCeilingMarginRatio;

  vector<cv::Point> contour;
  for (int vertex = 0; vertex < floorplan.GetNumRoomVertices(room); ++vertex) {
    const Vector2d local = floorplan.GetRoomVertexLocal(room, vertex);
    contour.push_back(cv::Point(local[0], local[1]));
  }
  
  segments->clear();
  segments->resize((int)points.size(), kInitial);
  // Floor check.
  for (int p = 0; p < (int)points.size(); ++p) {
    if (IsOnFloor(floorplan.GetFloorHeight(room), floor_margin, points[p]))
      segments->at(p) = kFloor;
    else if (IsOnCeiling(floorplan.GetCeilingHeight(room), ceiling_margin, points[p]))
      segments->at(p) = kCeiling;
    else if (IsOnWall(contour, wall_margin, points[p]))
      segments->at(p) = kWall;
  }
}

void FilterNoisyPoints(std::vector<Point>* points) {
  const int kNumNeighbors = 10;

  vector<float> point_data;
  {
    point_data.reserve(3 * points->size());
    for (int p = 0; p < points->size(); ++p) {
      for (int i = 0; i < 3; ++i)
        point_data.push_back(points->at(p).position[i]);
    }
  }
  KDtree kdtree(point_data);
  vector<const float*> knn;
  vector<float> neighbor_distances(points->size());
  for (int p = 0; p < points->size(); ++p) {
    knn.clear();
    const Vector3f ref_point(points->at(p).position[0],
                             points->at(p).position[1],
                             points->at(p).position[2]);
                      
    kdtree.find_k_closest_to_pt(knn, kNumNeighbors, &ref_point[0]);
                               
    double neighbor_distance = 0.0;
    for (int i = 0; i < knn.size(); ++i) {
      const float* fp = knn[i];
      const Vector3f point(fp[0], fp[1], fp[2]);
      neighbor_distance += (point - ref_point).norm();
    }
    neighbor_distances[p] = neighbor_distance / max(1, (int)knn.size());
  }

  //----------------------------------------------------------------------
  double average = 0.0;
  for (int p = 0; p < neighbor_distances.size(); ++p)
    average += neighbor_distances[p];
  average /= neighbor_distances.size();
  double deviation = 0.0;
  for (int p = 0; p < neighbor_distances.size(); ++p) {
    const double diff = neighbor_distances[p] - average;
    deviation += diff * diff;
  }
  deviation /= neighbor_distances.size();
  deviation = sqrt(deviation);

  const double threshold = average + 0.5 * deviation;
  vector<Point> new_points;
  for (int p = 0; p < neighbor_distances.size(); ++p) {
    if (neighbor_distances[p] <= threshold) {
      new_points.push_back(points->at(p));
    }
  }
  points->swap(new_points);
}

void Subsample(const double ratio, std::vector<Point>* points) {
  const int new_size = static_cast<int>(round(ratio * points->size()));
  random_shuffle(points->begin(), points->end());
  points->resize(new_size);
}
  
void SegmentObjects(const std::vector<Point>& points,
                    std::vector<int>* segments) {
  // Compute neighbors.
  vector<vector<int> > neighbors;
  const int kNumNeighbors = 30;
  SetNeighbors(points, kNumNeighbors, &neighbors);
  
  // Randomly initialize seeds.
  vector<int> seeds;
  {
    const int kNumInitialClusters = 60;
    for (int p = 0; p < segments->size(); ++p)
      if (segments->at(p) == kInitial)
        seeds.push_back(p);
    random_shuffle(seeds.begin(), seeds.end());
    seeds.resize(kNumInitialClusters);
  }

  // Initialize remaining.
  //vector<pair<double, int> > distance_segment(segments->size(), pair<double, int>(0.0, kInitial));

  priority_queue<pair<double, pair<int, int> > > distance_point_segment_queue;
  for (int c = 0; c < (int)seeds.size(); ++c) {
    const int point_index = seeds[c];
    // distance_segment[point_index] = pair<double, int>(0.0, c);
    const pair<int, int> point_segment(point_index, c);
    distance_point_segment_queue.push(pair<double, pair<int, int> >(0, point_segment));
  }

  while (!distance_point_segment_queue.empty()) {
    const auto distance_point_segment = distance_point_segment_queue.top();
    const double distance = - distance_point_segment.first;
    const int point_index = distance_point_segment.second.first;
    const int cluster     = distance_point_segment.second.second;
    distance_point_segment_queue.pop();

    if (segments->at(point_index) != kInitial)
      continue;

    segments->at(point_index) = cluster;
    
    for (int i = 0; i < neighbors[point_index].size(); ++i) {
      const int neighbor = neighbors[point_index][i];
      if (segments->at(neighbor) != kInitial)
        continue;

      const double new_distance = distance + PointDistance(points[point_index], points[neighbor]);
      distance_point_segment_queue.push(pair<double, pair<int, int> >(-new_distance,
                                                                      make_pair(neighbor, cluster)));
    }
  }
}
  
void SetNeighbors(const std::vector<Point>& points,
                  const int num_neighbors,
                  std::vector<std::vector<int> >* neighbors) {
  vector<float> point_data;
  {
    point_data.reserve(3 * points.size());
    for (int p = 0; p < points.size(); ++p) {
      for (int i = 0; i < 3; ++i)
        point_data.push_back(points[p].position[i]);
    }
  }
  KDtree kdtree(point_data);
  vector<const float*> knn;
  vector<float> neighbor_distances(points.size());

  neighbors->clear();
  neighbors->resize(points.size());

  for (int p = 0; p < points.size(); ++p) {
    knn.clear();
    const Vector3f ref_point(points[p].position[0],
                             points[p].position[1],
                             points[p].position[2]);
                      
    kdtree.find_k_closest_to_pt(knn, num_neighbors, &ref_point[0]);

    for (int i = 0; i < (int)knn.size(); ++i) {
      const int index = (knn[i] - &point_data[0]) / 3;
      neighbors->at(p).push_back(index);
    }
  }
}
  
}  // namespace structured_indoor_modeling
