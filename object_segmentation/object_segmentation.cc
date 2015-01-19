#include <Eigen/Dense>
#include <numeric>
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
  const Vector3d lhs_normal_diff = lhs.normal * lhs.normal.dot(diff);
  const Vector3d rhs_normal_diff = rhs.normal * rhs.normal.dot(diff);
  const Vector3d lhs_tangent_diff = diff - lhs_normal_diff;
  const Vector3d rhs_tangent_diff = diff - rhs_normal_diff;

  // Suppress distance along tangential.
  const double kSuppressRatio = 0.25;

  return kSuppressRatio * (lhs_tangent_diff.norm() + rhs_tangent_diff.norm()) +
    (lhs_normal_diff.norm() + rhs_normal_diff.norm());
}

void WritePointsWithColor(const std::vector<Point>& points,
                          const std::vector<int>& segments,
                          const std::string& filename) {
  vector<Point> object_points = points;
  for (int p = 0; p < object_points.size(); ++p) {
    switch (segments[p]) {
    case kFloor: {
      object_points[p].color = Vector3f(0, 0, 255);
      break;
    }
    case kWall: {
      object_points[p].color = Vector3f(0, 255, 0);
      break;
    }
    case kCeiling: {
      object_points[p].color = Vector3f(255, 0, 0);
      break;
    }
    default: {
      object_points[p].color[0] = segments[p] * 30 % 255;
      object_points[p].color[1] = segments[p] * 50 % 255;
      object_points[p].color[2] = segments[p] * 70 % 255;
    }
    }
  }

  PointCloud pc;
  pc.SetPoints(object_points);
  pc.Write(filename);
}

void WriteObjectPointsWithColor(const std::vector<Point>& points,
                                const std::vector<int>& segments,
                                const std::string& filename) {
  map<int, Vector3i> color_table;

  
  vector<Point> object_points;
  for (int p = 0; p < points.size(); ++p) {
    Point point = points[p];
    switch (segments[p]) {
    case kFloor:
    case kWall:
    case kCeiling: {
      break;
    }
    default: {
      if (color_table.find(segments[p]) == color_table.end()) {
        color_table[segments[p]][0] = rand() % 255;
        color_table[segments[p]][1] = rand() % 255;
        color_table[segments[p]][2] = rand() % 255;
      }
      
      point.color[0] = color_table[segments[p]][0];
      point.color[1] = color_table[segments[p]][1];
      point.color[2] = color_table[segments[p]][2];
      object_points.push_back(point);
      break;
    }
    }
  }
    
  PointCloud pc;
  pc.SetPoints(object_points);
  pc.Write(filename);
}

void InitializeCentroids(const int num_initial_clusters,
                         std::vector<int>* segments) {
  // Randomly initialize seeds.
  vector<int> seeds;
  {
    for (int p = 0; p < segments->size(); ++p)
      if (segments->at(p) == kInitial)
        seeds.push_back(p);
    random_shuffle(seeds.begin(), seeds.end());
    seeds.resize(num_initial_clusters);
  }

  for (int c = 0; c < (int)seeds.size(); ++c) {
    segments->at(seeds[c]) = c;
  }
}
  
const double kUnreachable = -1.0;
void ComputeDistances(const std::vector<Point>& points,
                      const std::vector<std::vector<int> >& neighbors,
                      const int index,
                      const std::vector<int>& segments,
                      std::vector<double>* distances) {
  distances->clear();
  distances->resize(points.size(), kUnreachable);
  
  priority_queue<pair<double, int> > distance_index_queue;
  distance_index_queue.push(make_pair(0.0, index));

  while (!distance_index_queue.empty()) {
    const auto distance_index = distance_index_queue.top();
    const double distance = -distance_index.first;
    const int point_index = distance_index.second;

    distance_index_queue.pop();
    // Already distance assigned.
    if (distances->at(point_index) != kUnreachable)
      continue;
    // ???? Stop at the boundary
    if (segments[point_index] == kFloor ||
        segments[point_index] == kWall ||
        segments[point_index] == kCeiling)
      continue;        

    distances->at(point_index) = distance;
    for (int i = 0; i < neighbors[point_index].size(); ++i) {
      const int neighbor = neighbors[point_index][i];
      if (distances->at(neighbor) != kUnreachable)
        continue;

      const double new_distance = distance + PointDistance(points[point_index], points[neighbor]);
      distance_index_queue.push(make_pair(- new_distance, neighbor));
    }
  }
}

void AssignFromCentroids(const std::vector<Point>& points,
                         const std::vector<std::vector<int> >& neighbors,
                         std::vector<int>* segments,
                         map<pair<int, int>, vector<double> >* cluster_pair_distances) {
  cluster_pair_distances->clear();
  // For each unassigned point, compute distance from centroids. Use
  // the top 25 percents average to assign to the closest centroid.
  const double kSumRatio = 0.25;

  vector<map<int, vector<double> > > point_cluster_distances(points.size());
  vector<double> distances;
  cerr << "Compute distances..." << flush;
  for (int p = 0; p < segments->size(); ++p) {
    if (p % 3000 == 0)
      cerr << '.' << flush;
    const int cluster = segments->at(p);
    if (cluster < 0)
      continue;

    ComputeDistances(points, neighbors, p, *segments, &distances);
    for (int p = 0; p < distances.size(); ++p) {
      if (distances[p] != kUnreachable) {
        point_cluster_distances[p][cluster].push_back(distances[p]);

        if (segments->at(p) >= 0) {
          const int target_cluster = segments->at(p);
          const pair<int, int> pair = make_pair(min(cluster, target_cluster),
                                                max(cluster, target_cluster));
          (*cluster_pair_distances)[pair].push_back(distances[p]);
        }        
      }
    }
  }
  cerr << "done." << endl;

  // For each point, compute the per cluster distance and pick the best.
  for (int p = 0; p < segments->size(); ++p) {
    if (segments->at(p) != kInitial)
      continue;

    const map<int, vector<double> >& cluster_distances = point_cluster_distances[p];
    double best_distance = 0.0;
    int best_cluster = kInitial;
    for (const auto& item : cluster_distances) {
      const int cluster        = item.first;
      vector<double> distances = item.second;
      if (distances.empty()) {
        cerr << "Impossible." << endl;
        exit (1);
      }
      sort(distances.begin(), distances.end());
      const int length_to_sum = max(1, static_cast<int>(round(distances.size() * kSumRatio)));
      double average_distance = 0.0;
      for (int i = 0; i < length_to_sum; ++i) {
        average_distance += distances[i];
      }
      average_distance /= length_to_sum;
      if (best_cluster == kInitial || average_distance <= best_distance) {
        best_cluster = cluster;
        best_distance = average_distance;
      }
    }
    segments->at(p) = best_cluster;
  }

  
  /*
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
  */
}  

void ComputeCentroids(const double ratio, vector<int>* segments) {
  map<int, vector<int> > cluster_to_centroids;
  for (int p = 0; p < segments->size(); ++p) {
    const int cluster = segments->at(p);
    if (cluster >= 0)
      cluster_to_centroids[cluster].push_back(p);
  }

  for (int p = 0; p < segments->size(); ++p) {
    const int cluster = segments->at(p);
    if (cluster >= 0)
      segments->at(p) = kInitial;
  }
  
  for (auto& cluster_to_centroid: cluster_to_centroids) {
    const int cluster = cluster_to_centroid.first;
    vector<int>& centroids = cluster_to_centroid.second;
    if (cluster == kInitial)
      continue;
    const int kMinimumCentroidSize = 2;
    if (centroids.size() < kMinimumCentroidSize)
      continue;
    
    random_shuffle(centroids.begin(), centroids.end());
    // We need at least 2 centroids so that intra cluster distance can be computed.
    const int new_size = max(kMinimumCentroidSize, static_cast<int>(round(centroids.size() * ratio)));
    centroids.resize(new_size);
    
    for (const auto centroid : centroids) {
      segments->at(centroid) = cluster;
    }
  }
}


void UnionFind(const std::set<std::pair<int, int> >& merged,
               const int max_old,
               std::map<int, int>* old_to_new) {
  vector<int> smallests(max_old);
  for (int old = 0; old < max_old; ++old) {
    const int kInvalid = -1;
    int smallest = old;
    for (const auto& merge : merged) {
      const int lhs = merge.first;
      const int rhs = merge.second;
      if (rhs == old && smallests[lhs] < smallest) {
        smallest = smallests[lhs];
      }
    }
    smallests[old] = smallest;
  }

  set<int> unique_ids;
  for (int i = 0; i < smallests.size(); ++i)
    unique_ids.insert(smallests[i]);

  map<int, int> unique_to_new;
  int new_id = 0;
  for (const auto& id : unique_ids) {
    unique_to_new[id] = new_id;
    ++new_id;
  }

  for (int i = 0; i < max_old; ++i) {
    if (unique_to_new.find(smallests[i]) == unique_to_new.end()) {
      cerr << "bug" << endl;
      exit (1);
    }
    (*old_to_new)[i] = unique_to_new[smallests[i]];
  }
}

void Merge(const std::vector<Point>& points,
           const std::vector<std::vector<int> >& neighbors,
           map<pair<int, int>, vector<double> >& cluster_pair_distances,
           std::vector<int>* segments) {
  map<pair<int, int>, vector<double> > cluster_pair_to_distances;
  for (int p0 = 0; p0 < (int)neighbors.size(); ++p0) {
    const int segment0 = segments->at(p0);
    for (int i = 0; i < (int)neighbors[p0].size(); ++i) {
      const int p1 = neighbors[p0][i];
      const int segment1 = segments->at(p1);

      cluster_pair_to_distances[make_pair(min(segment0, segment1), max(segment0, segment1))].
        push_back(PointDistance(points[p0], points[p1]));
    }
  }

  map<pair<int, int>, double> cluster_pair_to_average_distance;
  for (auto& item : cluster_pair_to_distances) {
    double average = 0.0;
    vector<double> distances = item.second;

    /*
    for (int i = 0; i < distances.size(); ++i)
      average += distances[i];
    average /= distances.size();
    */
    nth_element(distances.begin(),
                distances.begin() + distances.size() / 4,
                distances.end());
    cluster_pair_to_average_distance[item.first] = *(distances.begin() + distances.size() / 4);
  }

  double average_inter_distance = 0.0;
  {
    int denom = 0;
    for (auto& item : cluster_pair_to_distances) {
      if (item.first.first != item.first.second)
        continue;
      const vector<double>& distances = item.second;
      average_inter_distance += std::accumulate(distances.begin(), distances.end(), 0.0);
      denom += distances.size();
    }
    average_inter_distance /= max(1, denom);
  }
  cerr << "Average inter distance: " << average_inter_distance << endl;

  // Merge test.
  const double kMergeRatio = 2.0;
  set<pair<int, int> > merged;

  //pair<int, int> best_pair;
  //double best_ratio = 1000000;
  for (const auto& item : cluster_pair_to_distances) {
    const int p0 = item.first.first;
    const int p1 = item.first.second;
    if (p0 == p1)
      continue;
    const int size01 = item.second.size();
    const int size0 = cluster_pair_to_distances[make_pair(p0, p0)].size();
    const int size1 = cluster_pair_to_distances[make_pair(p1, p1)].size();

    const double distance01 = cluster_pair_to_average_distance[item.first];
    const double distance0  = cluster_pair_to_average_distance[make_pair(p0, p0)];
    const double distance1  = cluster_pair_to_average_distance[make_pair(p1, p1)];

    //?????
    //if (distance01 < distance0 * kMergeRatio && distance01 < distance1 * kMergeRatio)
    if (distance01 < average_inter_distance * kMergeRatio) {
      merged.insert(item.first);
      //?????? only one each.
      break;
    }
    /*
    const double ratio = distance01 / distance0 + distance01 / distance1;
    if (ratio < best_ratio) {
      best_ratio = ratio;
      best_pair = item.first;
    }
    */
  }
  // mamerged.insert(best_pair);

  cerr << "merged: " << merged.size() << " pairs." << endl;
  
  int max_cluster_id = 0;
  for (const auto& item : *segments) {
    max_cluster_id = max(max_cluster_id, item);
  }
  map<int, int> old_to_new;
  UnionFind(merged, max_cluster_id, &old_to_new);
  
  for (int i = 0; i < segments->size(); ++i) {
    if (segments->at(i) >= 0) {
      segments->at(i) = old_to_new[segments->at(i)];
    }
  }
  
  
  

  /*
  // Compute a list of possible clusters to be merged.
  map<pair<int, int>, double> averages;
  for (const auto& item : cluster_pair_distances) {
    const pair<int, int>& cluster_pair = item.first;
    double average = 0.0;
    for (const auto distance : item.second)
      average += distance;
    average /= item.second.size();
    averages[cluster_pair] = average;
  }

  set<pair<int, int> > merged;
  for (const auto& item : cluster_pair_distances) {
    const pair<int, int>& cluster_pair = item.first;
    const int lhs = cluster_pair.first;
    const int rhs = cluster_pair.second;

    if (averages.find(make_pair(lhs, lhs)) == averages.end()) {
      cerr << "Impossible." << endl;
    }
    if (averages.find(make_pair(rhs, rhs)) == averages.end()) {
      cerr << "Impossible." << endl;
    }
    
    if (averages[cluster_pair] < 2 * averages[make_pair(lhs, lhs)] &&
        averages[cluster_pair] < 2 * averages[make_pair(rhs, rhs)]) {
      merged.insert(cluster_pair);
    }
  }

  
  int max_cluster_id = 0;
  for (const auto& item : *segments) {
    max_cluster_id = max(max_cluster_id, item);
  }
  map<int, int> old_to_new;
  UnionFind(merged, max_cluster_id, &old_to_new);
  
  for (int i = 0; i < segments->size(); ++i) {
    if (segments->at(i) >= 0) {
      segments->at(i) = old_to_new[segments->at(i)];
    }
  }
  */
}
  
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
  const int kNumNeighbors = 20;

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
                    const double centroid_subsampling_ratio,
                    const int num_initial_clusters,
                    std::vector<int>* segments) {
  // Compute neighbors.
  vector<vector<int> > neighbors;
  const int kNumNeighbors = 8;
  cerr << "SetNeighbors..." << flush;
  SetNeighbors(points, kNumNeighbors, &neighbors);
  cerr << "done." << endl;

  WritePointsWithColor(points, *segments, "0_first.ply");
  InitializeCentroids(num_initial_clusters, segments);
  WritePointsWithColor(points, *segments, "1_init.ply");

  map<pair<int, int>, vector<double> > cluster_pair_distances;  
  AssignFromCentroids(points, neighbors, segments, &cluster_pair_distances);
  WriteObjectPointsWithColor(points, *segments, "2_seed.ply");


  // Repeat K-means.
  const int kTimes = 40;
  for (int t = 0; t < kTimes; ++t) {
    // Sample representative centers in each cluster.
    ComputeCentroids(centroid_subsampling_ratio, segments);

    // Assign remaining samples to clusters.
    AssignFromCentroids(points, neighbors, segments, &cluster_pair_distances);
    
    // Merge check.
    Merge(points, neighbors, cluster_pair_distances, segments);
    
    char buffer[1024];
    sprintf(buffer, "%d_ite.ply", 3 + t);
    WriteObjectPointsWithColor(points, *segments, buffer);
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
