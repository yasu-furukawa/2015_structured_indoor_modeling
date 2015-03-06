#include "../base/file_io.h"
#include "../base/panorama.h"
#include "generate_texture_indoor_polygon.h"
#include "synthesize.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

void PreparePatch(const TextureInput& texture_input,
                  const Segment& segment,
                  Patch* patch) {
  // Set patch_axes.
  const Vector3d kUpVector(0, 0, 1);
  
  switch (segment.normal) {
  case Segment::PositiveX: {
    patch->axes[2] = Vector3d(1, 0, 0);
    patch->axes[1] = kUpVector;
    break;
  }
  case Segment::NegativeX: {
    patch->axes[2] = Vector3d(-1, 0, 0);
    patch->axes[1] = kUpVector;
    break;
  }
  case Segment::PositiveY: {
    patch->axes[2] = Vector3d(0, 1, 0);
    patch->axes[1] = kUpVector;
    break;
  }
  case Segment::NegativeY: {
    patch->axes[2] = Vector3d(0, -1, 0);
    patch->axes[1] = kUpVector;
    break;
  }
  case Segment::PositiveZ: {
    patch->axes[2] = Vector3d(0, 0, 1);
    patch->axes[1] = Vector3d(1, 0, 0);
    break;
  }
  case Segment::NegativeZ: {
    patch->axes[2] = Vector3d(0, 0, -1);
    patch->axes[1] = Vector3d(1, 0, 0);
    break;
  }
  }
  patch->axes[0] = patch->axes[1].cross(patch->axes[2]);  

  // Set vertices.
  Vector3d min_xyz, max_xyz;
  for (int v = 0; v < segment.vertices.size(); ++v) {
    for (int i = 0; i < 3; ++i) {
      const double offset = segment.vertices[v].dot(patch->axes[i]);
      if (v == 0) {
        min_xyz[i] = max_xyz[i] = offset;
      } else {
        min_xyz[i] = min(min_xyz[i], offset);
        max_xyz[i] = max(max_xyz[i], offset);
      }
    }
  }
  if (min_xyz[2] != max_xyz[2]) {
    cerr << "Impossible: " << min_xyz[2] << ' ' << max_xyz[2] << endl;
  }
  const double z = min_xyz[2];
  patch->vertices[0] = patch->axes[0] * min_xyz[0] + patch->axes[1] * min_xyz[1] + patch->axes[2] * z;
  patch->vertices[1] = patch->axes[0] * max_xyz[0] + patch->axes[1] * min_xyz[1] + patch->axes[2] * z;
  patch->vertices[2] = patch->axes[0] * max_xyz[0] + patch->axes[1] * max_xyz[1] + patch->axes[2] * z;
  patch->vertices[3] = patch->axes[0] * min_xyz[0] + patch->axes[1] * max_xyz[1] + patch->axes[2] * z;
  
  // Set texture_size.
  int max_size;
  if (segment.type == Segment::FLOOR)
    max_size = texture_input.max_texture_size_per_floor_patch;
  else
    max_size = texture_input.max_texture_size_per_non_floor_patch;
  
  patch->texture_size[0] =
    min(max_size, max(2, static_cast<int>((max_xyz[0] - min_xyz[0]) / texture_input.texel_unit)));
  patch->texture_size[1] =
    min(max_size, max(2, static_cast<int>((max_xyz[1] - min_xyz[1]) / texture_input.texel_unit)));
}

int FindBestPanorama(const TextureInput& texture_input,
                     const Patch& patch) {
  const IndoorPolygon& indoor_polygon = texture_input.indoor_polygon;
  const std::vector<std::vector<Panorama> >& panoramas = texture_input.panoramas;
                          
  const double kHolePenalty = 0.5;
  const double kNormalScale = 0.5;
  const int kFirstLevel = 0;
  const int kInvalid = -1;
  int best_panorama = kInvalid;
  double best_score = 0.0;
  for (int p = 0; p < panoramas.size(); ++p) {
    const Vector3d& center = panoramas[p][kFirstLevel].GetCenter();
    double score = 0.0;
    // Sample points on the patch, and check the visibility for each panorama.
    const int kNumSamples1D = 10;
    for (int j = 0; j < kNumSamples1D; ++j) {
      const double v = (j + 0.5) / kNumSamples1D;
      for (int i = 0; i < kNumSamples1D; ++i) {
        const double u = (i + 0.5) / kNumSamples1D;
        const Vector3d sample =
          indoor_polygon.ManhattanToGlobal(patch.UVToManhattan(Vector2d(u, v)));
        Vector3d diff = center - sample;
        const double patch_distance = diff.norm();

        const Vector2d pixel = panoramas[p][kFirstLevel].Project(sample);
        if (panoramas[p][kFirstLevel].GetRGB(pixel) == Vector3f(0, 0, 0)) {
          score -= kHolePenalty;
        }

        const double dot =
          indoor_polygon.ManhattanToGlobal(patch.axes[2]).dot(diff.normalized());
        score += kNormalScale * dot;

        const Vector2d depth_pixel = panoramas[p][kFirstLevel].RGBToDepth(pixel);
        const double depth_distance = panoramas[p][kFirstLevel].GetDepth(depth_pixel);
        score += min(0.0,
                      (depth_distance - patch_distance) / panoramas[p][kFirstLevel].GetAverageDistance());
      }
    }
    score /= kNumSamples1D * kNumSamples1D;
    if (best_panorama == kInvalid || score > best_score) {
      best_score = score;
      best_panorama = p;
    }
  }

  if (best_panorama == kInvalid) {
    cerr << "Impossible." << endl;
    exit (1);
  }
  return best_panorama;
}  

int ChoosePyramidLevel(const IndoorPolygon& indoor_polygon,
                       const std::vector<Panorama>& panorama,
                       const Patch& patch) {
  const int kLevelZero = 0;
  const Vector3d center =
    indoor_polygon.ManhattanToGlobal(patch.UVToManhattan(Vector2d(0.5, 0.5)));
  const Vector3d center_right =
    indoor_polygon.ManhattanToGlobal(patch.UVToManhattan(Vector2d(0.5 + 1.0 / patch.texture_size[0], 0.5)));

  const Vector2d center_pixel = panorama[kLevelZero].Project(center);
  const Vector2d center_right_pixel = panorama[kLevelZero].Project(center_right);

  double pixel_diff = (center_right_pixel - center_pixel).norm();
  if (pixel_diff > panorama[kLevelZero].Width() / 2)
    pixel_diff = abs(pixel_diff - panorama[kLevelZero].Width());

  const int num_levels = panorama.size();
  const int level = max(0, min(num_levels - 1, static_cast<int>(floor(log(pixel_diff) / log(2)))));

  return level;
}
  
void GrabTexture(const IndoorPolygon& indoor_polygon,
                 const std::vector<Panorama>& panorama,
                 Patch* patch) {
  const int level = ChoosePyramidLevel(indoor_polygon, panorama, *patch);
  const Panorama& pano = panorama[level];

  patch->texture.clear();
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x) {
      const Vector3d manhattan =
        patch->UVToManhattan(Vector2d((x + 0.5) / patch->texture_size[0],
                                      (y + 0.5) / patch->texture_size[1]));
      const Vector3d& point = indoor_polygon.ManhattanToGlobal(manhattan);
      const Vector3f rgb = pano.GetRGB(pano.Project(point));
      for (int i = 0; i < 3; ++i) {
        patch->texture.push_back(static_cast<unsigned char>(round(rgb[i])));
      }
    }
  }
}

void UpdateIUV(const Eigen::Vector2i& size,
               const int texture_image_size,
               std::pair<int, Eigen::Vector2i>* iuv,
               int* max_texture_height) {
  // New row.
  if (iuv->second[0] + size[0] > texture_image_size) {
    iuv->second[0] = 0;
    iuv->second[1] += *max_texture_height;
    *max_texture_height = 0;
  }
  
  // New image.
  if (iuv->second[1] + size[1] > texture_image_size) {
    ++iuv->first;
    iuv->second[0] = 0;
    iuv->second[1] = 0;
    *max_texture_height = 0;
  }

  // Add a patch.
  *max_texture_height = max(*max_texture_height, size[1]);
}

void CopyTexel(const Patch& patch,
               const int texture_image_size,
               const std::pair<int, Eigen::Vector2i>& iuv,
               std::vector<std::vector<unsigned char> >* texture_images) {
  const int kNumChannels = 3;
  if (iuv.first >= texture_images->size()) {
    texture_images->push_back(vector<unsigned char>());
    texture_images->back().resize(kNumChannels * texture_image_size * texture_image_size);
  }
  
  const int index = iuv.first;
  const Vector2i& start_uv = iuv.second;

  if (texture_images->size() <= index) {
    cerr << "Impossible." << endl;
    exit (1);
  }

  for (int y = 0; y < patch.texture_size[1]; ++y) {
    const int ytmp = y + start_uv[1];
    for (int x = 0; x < patch.texture_size[0]; ++x) {
      const int xtmp = x + start_uv[0];
      for (int c = 0; c < kNumChannels; ++c) {
        const int texture_index = kNumChannels * (ytmp * texture_image_size + xtmp);
        const int patch_index = kNumChannels * (y * patch.texture_size[0] + x);
        texture_images->at(index)[texture_index + c] = patch.texture[patch_index + c];
      }
    }
  }
}

void SetIUVInSegment(const Patch& patch,
                     const int texture_image_size,
                     const std::pair<int, Eigen::Vector2i>& iuv,
                     Segment* segment) {
  const Vector2i& texture_size = patch.texture_size;
  Vector2d top_left_uv(iuv.second[0], iuv.second[1]);
  Vector2d bottom_right_uv(iuv.second[0] + texture_size[0],
                           iuv.second[1] + texture_size[1]);

  // Convert to [0, 1].
  top_left_uv /= texture_image_size;
  bottom_right_uv /= texture_image_size;
  Vector2d uv_diff = bottom_right_uv - top_left_uv;
  
  for (auto& triangle : segment->triangles) {
    triangle.image_index = iuv.first;
    for (int i = 0; i < 3; ++i) {      
      Vector2d uv_in_patch =
        patch.ManhattanToUV(segment->vertices[triangle.indices[i]]);

      // May need to change depending on the definition of uv(0, 0).
      triangle.uvs[i] = top_left_uv + Vector2d(uv_in_patch[0] * uv_diff[0],
                                               uv_in_patch[1] * uv_diff[1]);

      for (int j = 0; j < 2; ++j)
        triangle.uvs[i][j] = min(1.0, triangle.uvs[i][j]);
    }
  }
}

void ShrinkTexture(const int shrink_pixels, Patch* patch) {
  vector<bool> valids(patch->texture_size[0] * patch->texture_size[1], false);
  int index = 0;
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
      if (patch->texture[3 * index + 0] != 0 ||
          patch->texture[3 * index + 1] != 0 ||
          patch->texture[3 * index + 2] != 0) {
        valids[index] = true;
      }
    }
  }

  for (int t = 0; t < shrink_pixels; ++t) {
    vector<bool> valids_org = valids;
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y) {
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
        if (!valids_org[index]) {
          if ((x != 0                          && valids_org[index - 1]) ||
              (x != patch->texture_size[0] - 1 && valids_org[index + 1]) ||
              (y != 0                          && valids_org[index - patch->texture_size[0]]) ||
              (y != patch->texture_size[1] - 1 && valids_org[index + patch->texture_size[0]])) {
            valids[index] = true;
          }
        }
      }
    }
  }

  index = 0;
  for (int y = 0; y < patch->texture_size[1]; ++y) {
    for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
      if (!valids[index]) {
        patch->texture[3 * index + 0] = 0;
        patch->texture[3 * index + 1] = 0;
        patch->texture[3 * index + 2] = 0;
      }
    }
  }
}

void SynthesizePatch(Patch* patch) {
  vector<cv::Mat> projected_textures;
  cv::Mat projected_texture(patch->texture_size[1],
                            patch->texture_size[0],
                            CV_8UC3);
  {
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y)
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index)
        projected_texture.at<cv::Vec3b>(y, x) = cv::Vec3b(patch->texture[3 * index + 0],
                                                          patch->texture[3 * index + 1],
                                                          patch->texture[3 * index + 2]);
  }
  
  projected_textures.push_back(projected_texture);
  SynthesisData synthesis_data(projected_textures);

  synthesis_data.num_cg_iterations = 50;
  synthesis_data.texture_size = patch->texture_size;
  synthesis_data.patch_size = min(80, min(patch->texture_size[0], patch->texture_size[1]));
  synthesis_data.margin = synthesis_data.patch_size / 4;
  synthesis_data.mask.resize(patch->texture_size[0] * patch->texture_size[1], true);

  vector<cv::Mat> patches;
  vector<Eigen::Vector2i> patch_positions;
  CollectCandidatePatches(synthesis_data, &patches, &patch_positions);
  if (patches.empty())
    return;
  
  cv::Mat synthesized_texture(patch->texture_size[1],
                              patch->texture_size[0],
                              CV_8UC3,
                              cv::Scalar(0));
  const bool kVerticalConstraint = true;
  SynthesizePoisson(synthesis_data, patches, patch_positions, kVerticalConstraint,
                    &synthesized_texture);
  cv::imshow("result", synthesized_texture);

  {
    int index = 0;
    for (int y = 0; y < patch->texture_size[1]; ++y) {
      for (int x = 0; x < patch->texture_size[0]; ++x, ++index) {
        patch->texture[3 * index + 0] = synthesized_texture.at<cv::Vec3b>(y, x)[0];
        patch->texture[3 * index + 1] = synthesized_texture.at<cv::Vec3b>(y, x)[1];
        patch->texture[3 * index + 2] = synthesized_texture.at<cv::Vec3b>(y, x)[2];
      }
    }
  }
}
  
}  // namespace

Eigen::Vector3d Patch::UVToManhattan(const Eigen::Vector2d& uv) const {
  return vertices[0] + uv[0] * (vertices[1] - vertices[0]) + uv[1] * (vertices[3] - vertices[0]);
}

Eigen::Vector2d Patch::ManhattanToUV(const Eigen::Vector3d& manhattan) const {
  const double x_length = (vertices[1] - vertices[0]).norm();
  const double y_length = (vertices[3] - vertices[0]).norm();
  
  return Eigen::Vector2d(std::max(0.0, std::min(1.0, (manhattan - vertices[0]).dot(axes[0]) / x_length)),
                         std::max(0.0, std::min(1.0, (manhattan - vertices[0]).dot(axes[1]) / y_length)));
}
  
Eigen::Vector2d Patch::UVToTexture(const Eigen::Vector2d& uv) const {
  return Eigen::Vector2d(texture_size[0] * uv[0], texture_size[1] * uv[1]);
}
  
Eigen::Vector2d Patch::TextureToUV(const Eigen::Vector2d& texture) const {
  return Eigen::Vector2d(texture[0] / texture_size[0], texture[1] / texture_size[1]);
}

double ComputeTexelUnit(const IndoorPolygon& indoor_polygon,
                        const int target_texture_size_for_vertical) {
  // Collect z coordinates.
  vector<double> z_values;
  for (int s = 0; s < indoor_polygon.GetNumSegments(); ++s) {
    const Segment& segment = indoor_polygon.GetSegment(s);
    for (const auto& vertex : segment.vertices) {
      z_values.push_back(vertex[2]);
    }
  }

  // Take 3 and 97 percentiles.
  vector<double>::iterator floor_z_ite =
    z_values.begin() + z_values.size() * 3 / 100;
  vector<double>::iterator ceiling_z_ite =
    z_values.begin() + z_values.size() * 97 / 100;

  nth_element(z_values.begin(), floor_z_ite, z_values.end());
  const double floor_z = *floor_z_ite;
  
  nth_element(z_values.begin(), ceiling_z_ite, z_values.end());
  const double ceiling_z = *ceiling_z_ite;

  if (target_texture_size_for_vertical == 0) {
    cerr << "Impossible parameter: " << target_texture_size_for_vertical << endl;
    exit (1);
  }
  return (ceiling_z - floor_z) / target_texture_size_for_vertical;
}
  
void SetPatch(const TextureInput& texture_input,
              const Segment& segment,
              const bool visibility_check,
              Patch* patch) {
  PreparePatch(texture_input, segment, patch);

  // if (visibility_check) {
  //????
  if (0) {
    // Project from all the panoramas, and blend.
    
  } else {
    // Pick the best one and inpaint.
    const int best_panorama = FindBestPanorama(texture_input, *patch);
    GrabTexture(texture_input.indoor_polygon,
                texture_input.panoramas[best_panorama],
                patch);

    const int kShrinkPixels = 6;
    ShrinkTexture(kShrinkPixels, patch);

    bool hole = false;
    for (int i = 0; i < patch->texture.size(); i+=3) {
      if (patch->texture[i] == 0 &&
          patch->texture[i + 1] == 0 &&
          patch->texture[i + 2] == 0) {
        hole = true;
        break;
      }
    }
    if (hole)
      SynthesizePatch(patch);    
  }  
}

void PackTexture(const Patch& patch,
                 const int texture_image_size,
                 Segment* segment,
                 std::vector<std::vector<unsigned char> >* texture_images,
                 std::pair<int, Eigen::Vector2i>* iuv,
                 int* max_texture_height) {
  UpdateIUV(patch.texture_size, texture_image_size, iuv, max_texture_height);
  // Copy texel data.
  CopyTexel(patch, texture_image_size, *iuv, texture_images);
  // Update IUV in segment.
  SetIUVInSegment(patch, texture_image_size, *iuv, segment);
  // Update iuv.
  iuv->second[0] += patch.texture_size[0];
}

void WriteTextureImages(const FileIO& file_io,
                        const int texture_image_size,
                        const std::vector<std::vector<unsigned char> >& texture_images) {
  for (int t = 0; t < texture_images.size(); ++t) {
    cv::Mat image(texture_image_size, texture_image_size, CV_8UC3);
    int index = 0;
    for (int y = 0; y < texture_image_size; ++y) {
      for (int x = 0; x < texture_image_size; ++x) {
        for (int i = 0; i < 3; ++i) {
          image.at<cv::Vec3b>(y, x)[i] = texture_images[t][index++];
        }
      }
    }

    imwrite(file_io.GetTextureImageIndoorPolygon(t), image);
  }
}

}  // namespace structured_indoor_modeling
