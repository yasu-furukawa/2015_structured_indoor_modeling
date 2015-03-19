#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "detection.h"

namespace structured_indoor_modeling {

typedef std::pair<int, int> ObjectId;

void RasterizeObjectIds(const std::vector<Panorama>& panoramas,
                        const std::vector<PointCloud>& object_point_clouds,
                        std::vector<std::vector<ObjectId> >* object_ids);
 
void AssociateObjectId(const std::vector<Panorama>& panoramas,
                       const std::vector<Detection>& detections,
                       const std::vector<std::vector<ObjectId> >& object_ids,
                       const double score_threshold,
                       const double area_threshold,
                       std::map<ObjectId, Detection>* object_id_to_detection);

}  // namespace structured_indoor_modeling
