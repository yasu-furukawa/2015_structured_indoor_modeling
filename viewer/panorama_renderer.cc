#include <fstream>
#include <iostream>
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

//void MyColor(const Eigen::Vector3d& vertex) {
/*
  const int kUnit = 50;

  const float red = (static_cast<int>(vertex[0]) % kUnit) / static_cast<float>(kUnit);
  const float green = (static_cast<int>(vertex[1]) % kUnit) / static_cast<float>(kUnit);
  const float blue = (static_cast<int>(vertex[2]) % kUnit) / static_cast<float>(kUnit);

  glColor3f(red, green, blue);
  */
//}

void PanoramaRenderer::Render(const double alpha) {
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    //glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    const int kSkip = 1;

    glBegin(GL_TRIANGLES);
    // glColor4f(1, 1, 1, alpha);
    glColor4f(alpha, alpha, alpha, 1.0);
    //for (int y = 0; y < depth_height - 1; ++y) {
    //for (int x = 0; x < depth_width; ++x) {
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

void PanoramaRenderer::Init(const PanoramaConfiguration& configuration,
                            QGLWidget* widget_tmp) {
    widget = widget_tmp;
    rgb_image.load(configuration.rgb_image_filename.c_str());
    if (rgb_image.isNull()) {
        cout<<"Can not open image file"<<endl;
        exit (1);
    }
    phi_per_pixel = configuration.phi_range / rgb_image.height();

    center = configuration.center;
    local_to_global = configuration.local_to_global;
    global_to_local = local_to_global.transpose();

    InitDepthMesh(configuration.depth_image_filename, configuration.phi_range);
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

// u coordinate is guaranteed to be in [0, width).
// v coordinate could be outside the range [0, height].
Vector2d PanoramaRenderer::Project(const Vector3d& xyz) const {
    Vector3d projected_coordinate = global_to_local * (xyz - center);
    // x coordinate.
    double theta = -atan2(projected_coordinate.y(), projected_coordinate.x());
    if (theta < 0.0)
        theta += 2 * M_PI;
    double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
    if (theta_ratio == 1.0)
        theta_ratio = 0.0;

    Vector2d uv;
    uv[0] = theta_ratio * rgb_image.width();
    const double depth = sqrt(projected_coordinate.x() * projected_coordinate.x() +
                              projected_coordinate.y() * projected_coordinate.y());
    double phi = atan2(projected_coordinate.z(), depth);
    const double pixel_offset_from_center = phi / phi_per_pixel;
    uv[1] = rgb_image.height() / 2.0 - pixel_offset_from_center;

    return uv;
}

Vector3d PanoramaRenderer::Unproject(const Vector2d& uv, const double distance) const {
    //const double theta = -2.0 * M_PI * uv[0] / rgb_image.width();
    //const double phi   = (rgb_image.height() / 2.0 - uv[1]) * phi_per_pixel;
    const double theta = -2.0 * M_PI * uv[0] / depth_width;
    const double phi   = (depth_height / 2.0 - uv[1]) * depth_phi_per_pixel;

    Vector3d xyz;
    xyz[2] = distance * sin(phi);
    xyz[0] = distance * cos(phi) * cos(theta);
    xyz[1] = distance * cos(phi) * sin(theta);

    return xyz;
}

Eigen::Vector3d PanoramaRenderer::GlobalToLocal(const Eigen::Vector3d& global_xyz) const {
    return global_to_local * (global_xyz - center);
}

Eigen::Vector3d PanoramaRenderer::LocalToGlobal(const Eigen::Vector3d& local_xyz) const {
    return local_to_global * local_xyz + center;
}

void PanoramaRenderer::InitDepthMesh(const string& filename, const double phi_range) {
    ifstream ifstr;
    ifstr.open(filename.c_str());

    string header;
    double min_depth, max_depth;
    ifstr >> header >> depth_width >> depth_height >> min_depth >> max_depth;

    depth_mesh.resize(depth_width * depth_height);
    depth_phi_per_pixel = phi_range / (double)depth_height;

    average_distance = 0.0;
    int denom = 0;
    for (int y = 0; y < depth_height; ++y) {
        for (int x = 0; x < depth_width; ++x) {
            double distance;
            ifstr >> distance;
            average_distance += distance;
            ++denom;

            depth_mesh[y * depth_width + x] = LocalToGlobal(Unproject(Vector2d(x, y), distance));
        }
    }
    ifstr.close();

    if (denom == 0) {
        cerr << "Impossible in initdepthmesh: " << filename << endl;
        exit (1);
    }
    average_distance /= denom;
}

Eigen::Vector2d PanoramaRenderer::RGBToDepth(const Eigen::Vector2d& pixel) const {
    return Vector2d(pixel[0] * depth_width / rgb_image.width(),
            pixel[1] * depth_height / rgb_image.height());
}

}  // namespace structured_indoor_modeling
