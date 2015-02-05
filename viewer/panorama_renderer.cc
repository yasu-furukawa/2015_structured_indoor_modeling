#include <fstream>
#include <iostream>
#include "../base/panorama.h"
#include "panorama_renderer.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

PanoramaRenderer::PanoramaRenderer() {
    texture_id = -1;
}

PanoramaRenderer::~PanoramaRenderer() {
    if (texture_id != -1)
        widget->deleteTexture(texture_id);
}

void PanoramaRenderer::Render(const double alpha) const {
  glBindTexture(GL_TEXTURE_2D, texture_id);
  
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  const int kSkip = 1;
  
  glBegin(GL_TRIANGLES);
  glColor4f(alpha, alpha, alpha, 1.0);
  for (int y = 0; y < depth_height - kSkip - 1; y += kSkip) {
    for (int x = 0; x < depth_width; x += kSkip) {
      const int right_x = (x + kSkip) % depth_width;
      const int index00 = y * depth_width + x;
      const int index01 = y * depth_width + right_x;
      const int index10 = (y + kSkip) * depth_width + x;
      const int index11 = (y + kSkip) * depth_width + right_x;
      // 00
      glTexCoord2d(x / static_cast<double>(depth_width),
                   1.0 - y / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index00][0], depth_mesh[index00][1], depth_mesh[index00][2]);
      // 10
      glTexCoord2d(x / static_cast<double>(depth_width),
                   1.0 - (y + kSkip) / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index10][0], depth_mesh[index10][1], depth_mesh[index10][2]);
      // 01
      glTexCoord2d((x + kSkip) / static_cast<double>(depth_width),
                   1.0 - y / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index01][0], depth_mesh[index01][1], depth_mesh[index01][2]);
      // 10
      glTexCoord2d(x / static_cast<double>(depth_width),
                   1.0 - (y + kSkip) / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index10][0], depth_mesh[index10][1], depth_mesh[index10][2]);
      // 11
      glTexCoord2d((x + kSkip) / static_cast<double>(depth_width),
                   1.0 - (y + kSkip) / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index11][0], depth_mesh[index11][1], depth_mesh[index11][2]);
      // 01
      glTexCoord2d((x + kSkip) / static_cast<double>(depth_width),
                   1.0 - y / static_cast<double>(depth_height));
      glVertex3d(depth_mesh[index01][0], depth_mesh[index01][1], depth_mesh[index01][2]);
    }
  }
  
  glEnd();
}
  
void PanoramaRenderer::Init(const FileIO& file_io,
                            const int panorama_id,
                            const Panorama* panorama_tmp,
                            QGLWidget* widget_tmp) {
  panorama = panorama_tmp;
  widget = widget_tmp;
  rgb_image.load(file_io.GetPanoramaImage(panorama_id).c_str());
  if (rgb_image.isNull()) {
    cout << file_io.GetPanoramaImage(panorama_id) << endl;
    exit (1);
  }

  InitDepthMesh(file_io.GetDepthPanorama(panorama_id), panorama->GetPhiRange());
}

void PanoramaRenderer::InitGL() {
    initializeGLFunctions();

    glEnable(GL_TEXTURE_2D);
    texture_id = widget->bindTexture(rgb_image);

    // Set nearest filtering mode for texture minification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Set bilinear filtering mode for texture magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}
  
void PanoramaRenderer::InitDepthMesh(const string& filename, const double phi_range) {
  ifstream ifstr;
  ifstr.open(filename.c_str());
  
  string header;
  double min_depth, max_depth;
  ifstr >> header >> depth_width >> depth_height >> min_depth >> max_depth;
  
  depth_mesh.resize(depth_width * depth_height);
  
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x) {
      double distance;
      ifstr >> distance;
      
      const Vector2d pixel = panorama->DepthToRGB(Vector2d(x, y));
      depth_mesh[y * depth_width + x] = panorama->Unproject(pixel, distance);
    }
  }
  ifstr.close();
}

}  // namespace structured_indoor_modeling
