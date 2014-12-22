#include "evidence.h"

#include <fstream>
#include <iostream>
#include <set>

using namespace Eigen;
using namespace std;

namespace floored {

// 0: floored first full
// 4: floored cvpr14
// 1: floored first [0.6, 0.8]
// 2: henry
// 3: rhouse
  const int dataset = 4;
  

double ScalePointEvidence(const double input) {
  // Full.
  double kGoodPointEvidence;
  switch (dataset) {
  case 0: {
    kGoodPointEvidence = 5.0;
    break;
  }
  case 4: {
    kGoodPointEvidence = 10.0;
    break;
  }
  case 1: {
    kGoodPointEvidence = 1.0;
    break;
  }
  case 2: {
    kGoodPointEvidence = 20.0;
    break;
  }
  default: {
    const double kBalloon = 10.0;
    kGoodPointEvidence = 1.0;
    const double scaled_input = input / kGoodPointEvidence;
    return 200.0 * min(2.5, max(0.0, scaled_input)) + kBalloon;
    
    break;
  }
  }

  const double scaled_input = input / kGoodPointEvidence;
  return 100.0 * min(2.55, max(0.0, scaled_input - 0.5));
}

double ScaleFreeSpaceEvidence(const double input) {
  double kGoodFreeSpaceEvidence;

  switch (dataset) {
  case 0: {
    kGoodFreeSpaceEvidence = 40.0;
    break;
  }
  case 4: {
    kGoodFreeSpaceEvidence = 100.0;
    break;
  }
  case 1: {
    kGoodFreeSpaceEvidence = 40.0;
    break;
  }
  case 2: {
    kGoodFreeSpaceEvidence = 2000.0;
    break;
  }
  default: {
    kGoodFreeSpaceEvidence = 10.0;
    break;
  }
  }

  const double scaled_input = input / kGoodFreeSpaceEvidence;
  return 100.0 * min(2.55, max(0.0, scaled_input - 0.5));
}


void ConvertEvidence(const int width,
                     const int height,
                     const std::vector<float>& evidence,
                     const double scale,
                     std::vector<unsigned char>* evidence_int) {
  evidence_int->clear();
  for (const auto& pixel : evidence) {
    const int gray_scale = min(255, static_cast<int>(scale * pixel));
    int red, green, blue;
    if (gray_scale < 128) {
      green  = gray_scale * 2;
      blue = 255 - green;
      red   = 0;
    } else {
      blue  = 0;
      red   = (gray_scale - 128) * 2;
      green = 255 - red;
    }
    evidence_int->push_back(static_cast<unsigned char>(red));
    evidence_int->push_back(static_cast<unsigned char>(green));
    evidence_int->push_back(static_cast<unsigned char>(blue));
  }
  
}
  
void LoadEvidence(const string& filename, vector<float>* evidence) {
  ifstream ifstr;
  ifstr.open(filename.c_str());

  int length;
  ifstr.read(reinterpret_cast<char*>(&length), sizeof(int));

  evidence->resize(length);
  ifstr.read(reinterpret_cast<char*>(&evidence->at(0)), sizeof(float) * length);

  ifstr.close();
}

void WriteEvidence(const string& filename, const vector<float>& evidence) {
  ofstream ofstr;
  ofstr.open(filename.c_str());

  int length = evidence.size();
  ofstr.write(reinterpret_cast<char*>(&length), sizeof(int));
  ofstr.write(reinterpret_cast<const char*>(&evidence[0]), sizeof(float) * length);

  ofstr.close();
}

void SetPointEvidence(const std::vector<Sweep>& sweeps,
                      const Frame& frame,
                      const std::string& directory,
                      std::vector<float>* point_evidence) {
  const int width  = frame.size[0];
  const int height = frame.size[1];
  
  point_evidence->clear();
  point_evidence->resize(width * height, 0.0f);

  /*
  // Draw points.
  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      const int x = static_cast<int>(round((point.position[0])));
      const int y = static_cast<int>(round((point.position[1])));
      if (0 <= x && x < width && 0 <= y && y < height) {
        double dot_product= fabs(point.normal.dot(frame.axes[2]));
        dot_product = max(0.0, 2.0 * (dot_product - 0.5));
        point_evidence->at(y * width + x) += max(1.0, 1.0 - dot_product) * point.weight;
      }
    }
  }
*/
  const float kStep = 0.5;
  const int kFirstStep = 4;
  const int kLastStep  = 10;

  for (const auto& sweep : sweeps) {
    for (const auto& point : sweep.points) {
      Vector2f position(point.position[0], point.position[1]);

      double dot_product;
      Vector2f normal(point.normal[0], point.normal[1]);
      if (point.normal == Vector3f(0, 0, 0)) {
        dot_product = 1.0;
      } else {
        normal.normalize();
        
        dot_product= fabs(point.normal.dot(frame.axes[2]));
        dot_product = max(0.0, 2.0 * (dot_product - 0.5));
      }
      const double weight = min(1.0, 1.0 - dot_product) * point.weight;
      set<pair<int, int> > pixels;

      for (int i = kFirstStep; i < kLastStep; ++i) {
        const Vector2f ptmp = position - (i * kStep) * normal;
        const int x = static_cast<int>(round(ptmp[0]));
        const int y = static_cast<int>(round(ptmp[1]));
        pixels.insert(make_pair(x, y));
      }
      for (const auto& pixel : pixels) {
        const int x = pixel.first;
        const int y = pixel.second;
        if (0 <= x && x < width && 0 <= y && y < height) {
          point_evidence->at(y * width + x) += weight;
        }
      }
    }
  }
}

void SetFreeSpaceEvidence(const std::vector<Sweep>& sweeps,
                          const Frame& frame,
                          const std::string& directory,
                          std::vector<float>* free_space_evidence) {
  const int width  = frame.size[0];
  const int height = frame.size[1];

  const int kFreeSpaceMargin = 20;
  free_space_evidence->clear();
  free_space_evidence->resize(width * height, 0.0f);

  int scount = 0;
  for (const auto& sweep : sweeps) {
    cerr << "sweep-count: " << scount++ << ' ' << sweep.points.size() << endl;

    int scount = 0;
    for (const auto& point : sweep.points) {
      // Draw a line.
      const Vector2f start(sweep.center[0], sweep.center[1]);
      const Vector2f goal(point.position[0], point.position[1]);

      //????
      // Ignores points outside the bounding box.
      if (goal[0] < 0 || width - 1 < goal[0] || goal[1] < 0 || height - 1 < goal[1]) {
        continue;
      }

      const Vector2f diff = goal - start;
      if (diff.norm() == 0.0) {
        continue;
      }
      Vector2f step = diff;
      step.normalize();
      step /= 2.0;
      if (step.norm() == 0.0) {
        continue;
      }
      
      const int step_num =
        max(0, min(2 * width, min(2 * height, static_cast<int>(floor(diff.norm() / step.norm())))));
      set<pair<int, int> > pixels;

      for (int i = 0; i < step_num - kFreeSpaceMargin; ++i) {
        const Vector2f pos = start + i * step;
        const int ix = static_cast<int>(round(pos[0]));
        const int iy = static_cast<int>(round(pos[1]));
        if (0 <= ix && ix < width && 0 <= iy && iy < height) {
          pixels.insert(make_pair<int, int>(ix, iy));
        }
      }
      for (const auto pixel : pixels) {
        (*free_space_evidence)[pixel.second * width + pixel.first] += point.weight;
      }
    }
  }
}  
  
void DrawEvidenceToImage(const Frame& frame,
                         const std::string directory,
                         const std::vector<float>& point_evidence,
                         const std::vector<float>& free_space_evidence) {
  vector<float> point_evidence_tmp(point_evidence.size());
  vector<float> free_space_evidence_tmp(free_space_evidence.size());;
  
  for (int i = 0; i < point_evidence.size(); ++i) {
    point_evidence_tmp[i] = ScalePointEvidence(point_evidence[i]);
    free_space_evidence_tmp[i] = ScaleFreeSpaceEvidence(free_space_evidence[i]);
  }    

  vector<unsigned char> point_evidence_int, free_space_evidence_int;
  {
    const double kPointScale = 1.0;
    ConvertEvidence(frame.size[0],
                    frame.size[1],
                    point_evidence_tmp,
                    kPointScale,
                    &point_evidence_int);
    
    ofstream ofstr;
    ofstr.open((directory + "point.ppm").c_str());
    ofstr << "P3" << endl
          << frame.size[0] << ' ' << frame.size[1] << endl
          << 255 << endl;
    for (const auto& intensity : point_evidence_int) {
      ofstr << static_cast<int>(intensity) << ' ';
    }
    ofstr.close();
  }
  
  {
    const double kFreeSpaceScale = 1.0;
    ConvertEvidence(frame.size[0],
                    frame.size[1],
                    free_space_evidence_tmp,
                    kFreeSpaceScale,
                    &free_space_evidence_int);
    
    ofstream ofstr;
    ofstr.open((directory + "free_space.ppm").c_str());
    ofstr << "P3" << endl
          << frame.size[0] << ' ' << frame.size[1] << endl
          << 255 << endl;
    for (const auto& intensity : free_space_evidence_int) {
      ofstr << static_cast<int>(intensity) << ' ';
    }
    ofstr.close();
  }
}

}  // namespace floored
