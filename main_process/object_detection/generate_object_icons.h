#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <list>
#include "../../base/detection.h"
#include "../../base/panorama.h"
#include "../../base/point_cloud.h"

namespace structured_indoor_modeling {

class IndoorPolygon;
// room, object
typedef std::pair<int, int> ObjectId;

void RasterizeObjectIds(const std::vector<Panorama>& panoramas,
                        const std::vector<PointCloud>& object_point_clouds,
                        std::vector<std::vector<ObjectId> >* object_ids);
 
void AssociateObjectId(const std::vector<Panorama>& panoramas,
                       const std::vector<Detection>& detections,
                       const std::vector<std::vector<ObjectId> >& object_ids,
                       const double score_threshold,
                       const double area_threshold,
                       std::map<ObjectId, int>* object_id_to_detection);

void AddIconInformationToDetections(const IndoorPolygon& indoor_polygon,
                                    const std::vector<PointCloud>& object_point_clouds,
                                    const std::map<ObjectId, int>& object_to_detection,
                                    std::vector<Detection>* detections);

void ComputeObjectPolygon(const std::vector<Eigen::Vector3d>& object_points,
			  Detection &detection);

void MarchingCube(std::vector<std::vector<double> >&grid,
		  std::vector<Eigen::Vector2d>&vlist,
		  std::vector<Eigen::Vector2i>&elist,
		  const double isovalue);

void Smoothing(std::vector<Eigen::Vector2d>& vlist, const int iteration = 1, const double lambda = 0.5);
void Simplification(std::vector<Eigen::Vector2d>& vlist, const int target = 20, const double margin = 10.0);

bool SanityCheck(const std::vector<Eigen::Vector2i>&elist);

void SortPolygon(std::vector<Eigen::Vector2d>& vlist,std::vector<Eigen::Vector2i>&elist);
		 


}  // namespace structured_indoor_modeling
