#pragma once

#include <Eigen/Dense>
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
                       std::vector<ObjectId>* associated_object_ids);

}  // namespace structured_indoor_modeling
