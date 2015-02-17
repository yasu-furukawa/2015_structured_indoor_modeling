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
  /*
  if (!program.bind()) {
    cerr << "Cannot bind." << endl;
    exit (1);
  }
  program.setUniformValue("phi_range", static_cast<float>(panorama->GetPhiRange()));

  GLfloat[4][4] global_to_local;
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      global_to_local[y][x] = panorama->GetGlobalToLocal()(x, y);
    }
  }
  program.setUniformValue("global_to_local", global_to_local);
  program.setUniformValue("tex0", 0);

  glActiveTexture(GL_TEXTURE0);
  */
  glBindTexture(GL_TEXTURE_2D, texture_id);
  glEnable(GL_TEXTURE_2D);
  
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  glBegin(GL_TRIANGLES);
  glColor4f(alpha, alpha, alpha, 1.0);
  for (int y = 0; y < depth_height - 1; ++y) {
    for (int x = 0; x < depth_width; ++x) {
      const int right_x = (x + 1) % depth_width;
      const int index00 = y * depth_width + x;
      const int index01 = y * depth_width + right_x;
      const int index10 = (y + 1) * depth_width + x;
      const int index11 = (y + 1) * depth_width + right_x;

      const Vector3d v00 = depth_mesh[index00];
      const Vector3d v10 = depth_mesh[index10];
      const Vector3d v01 = depth_mesh[index01];
      const Vector3d v11 = depth_mesh[index11];
      
      // 00
      glTexCoord2d((x + 0.5) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5) / static_cast<double>(depth_height));
      glVertex3d(v00[0], v00[1], v00[2]);
      // 10
      glTexCoord2d((x + 0.5) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5 + 1) / static_cast<double>(depth_height));
      glVertex3d(v10[0], v10[1], v10[2]);
      // 01
      glTexCoord2d((x + 0.5 + 1) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5) / static_cast<double>(depth_height));
      glVertex3d(v01[0], v01[1], v01[2]);

      // 10
      glTexCoord2d((x + 0.5) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5 + 1) / static_cast<double>(depth_height));
      glVertex3d(v10[0], v10[1], v10[2]);
      // 11
      glTexCoord2d((x + 0.5 + 1) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5 + 1) / static_cast<double>(depth_height));
      glVertex3d(v11[0], v11[1], v11[2]);
      // 01
      glTexCoord2d((x + 0.5 + 1) / static_cast<double>(depth_width),
                   1.0 - (y + 0.5) / static_cast<double>(depth_height));
      glVertex3d(v01[0], v01[1], v01[2]);
    }
  }
  glEnd();

  // program.release();
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
  /*
  ifstream ifstr;
  ifstr.open(filename.c_str());
  
  string header;
  double min_depth, max_depth;
  ifstr >> header >> depth_width >> depth_height >> min_depth >> max_depth;

  vector<double> depths;
  depths.reserve(depth_width * depth_height);
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x) {
      double distance;
      ifstr >> distance;
      depths.push_back(distance);
    }
  }
  ifstr.close();
  */

  depth_width  = panorama->DepthWidth();
  depth_height = panorama->DepthHeight();

  depth_mesh.resize(depth_width * depth_height);
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x) {
      const Vector2d pixel = panorama->DepthToRGB(Vector2d(x + 0.5, y + 0.5));
      depth_mesh[y * depth_width + x] = panorama->Unproject(pixel, panorama->GetDepth(Vector2d(x, y)));
    }
  }
}

}  // namespace structured_indoor_modeling
