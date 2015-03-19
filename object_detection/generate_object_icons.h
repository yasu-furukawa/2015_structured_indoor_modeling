#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "detection.h"

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
                                    const std::vector<Detection>& detections,
                                    std::vector<Detection>* detections_with_icon);

}  // namespace structured_indoor_modeling
