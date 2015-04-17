#include <iostream>
#include <fstream>
#include <limits>
#include <vector>

#include <gflags/gflags.h>

#include "../../base/file_io.h"
#include "../../base/floorplan.h"
#include "../../base/indoor_polygon.h"
#include "../../base/panorama.h"
#include "../../base/point_cloud.h"
#include "evaluate.h"

#ifdef _WIN32
#pragma comment (lib, "gflags.lib") 
#pragma comment (lib, "Shlwapi.lib") 
#endif

DEFINE_string(floorplan_file, "", "Floorplan filename.");
DEFINE_string(indoor_polygon_file, "", "Indoor_polygon filename.");

DEFINE_bool(evaluate_floorplan, false, "Evaluate floorplan.");
DEFINE_bool(evaluate_indoor_polygon, false, "Evaluate indoor_polygon.");
DEFINE_bool(evaluate_indoor_polygon_and_object_point_clouds, false, "Evaluate indoor_polygon.");
DEFINE_bool(evaluate_poisson_mesh, false, "Evaluate poisson mesh.");
DEFINE_bool(evaluate_vgcut_mesh, false, "Evaluate poisson mesh.");

DEFINE_bool(evaluate_all, false, "Evaluate all.");

using namespace Eigen;
using namespace std;
using namespace structured_indoor_modeling;

namespace {

void WriteDepthmap(const Panorama& panorama,
                   const std::vector<RasterizedGeometry>& rasterized_geometry,
                   const double invalid_depth,
                   const string& filename) {
  const int width  = panorama.DepthWidth();
  const int height = panorama.DepthHeight();

  bool first = true;
  double max_depth, min_depth;
  for (int i = 0; i < rasterized_geometry.size(); ++i) {
    if (rasterized_geometry[i].depth != invalid_depth) {
      if (first) {
        max_depth = min_depth = rasterized_geometry[i].depth;
        first = false;
      } else {
        max_depth = max(max_depth, rasterized_geometry[i].depth);
        min_depth = min(min_depth, rasterized_geometry[i].depth);
      }
    }
  }  

  ofstream ofstr;
  ofstr.open(filename.c_str());
  ofstr << "P3" << endl
        << width << ' ' << height << endl
        << 255 << endl;
  for (int i = 0; i < rasterized_geometry.size(); ++i) {
    if (rasterized_geometry[i].depth == invalid_depth) {
      ofstr << "255 0 0 ";
    } else {
      const int itmp = (int)(255 * (rasterized_geometry[i].depth - min_depth) / (max_depth - min_depth));
      ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
    }
  }
  ofstr.close();

  //
  /*
  {
    ofstream ofstr;
    ofstr.open("test.obj");
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int index = y * width + x;
        
        if (rasterized_geometry[index].depth == invalid_depth)
          continue;

        Vector2d depth_pixel(x, y);
        Vector2d pixel = panorama.DepthToRGB(depth_pixel);
        Vector3d global = panorama.Unproject(pixel, rasterized_geometry[index].depth);
        ofstr << "v " << global[0] << ' ' << global[1] << ' ' << global[2] << endl;
      }
    }
    ofstr.close();
  }
  */
}

void WriteDepthErrormap(const PointCloud& input_point_cloud,
                        const std::vector<RasterizedGeometry>& rasterized_geometry,
                        const Panorama& panorama,
                        const double invalid_depth,
                        const double depth_unit,
                        const string& depth_error_filename,
                        const string& normal_error_filename) {
  const int width  = panorama.DepthWidth();
  const int height = panorama.DepthHeight();
  Panorama small_panorama = panorama;
  small_panorama.Resize(Vector2i(width, height));
  const double kInvalid = -1.0;
  vector<double> depth_errors(width * height, kInvalid);
  vector<double> normal_errors(width * height, kInvalid);
  
  for (int p = 0; p < (int)input_point_cloud.GetNumPoints(); ++p) {
    const Point& point = input_point_cloud.GetPoint(p);
    const Vector2d pixel = panorama.ProjectToDepth(point.position);
    const int u = max(0, min(width - 1, static_cast<int>(round(pixel[0]))));
    const int v = max(0, min(height - 1, static_cast<int>(round(pixel[1]))));
    const int index = v * width + u;
    
   // No rasterized geometry. Very unlikely...
    if (rasterized_geometry[index].depth == invalid_depth) {
      // cerr << "Rendering hole. This should rarely happen." << endl;
      continue;
    }
    
    const double depth_error = 
      fabs(rasterized_geometry[index].depth - (panorama.GetCenter() - point.position).norm());
    const double normal_error =
      acos(min(1.0, max(-1.0, rasterized_geometry[index].normal.dot(point.normal)))) * 180.0 / M_PI;
    
    depth_errors[index] = depth_error;
    normal_errors[index] = normal_error;
  }

  const double alpha = 0.0;
  const double kMaxDepthError = 0.125;
  {
    ofstream ofstr;
    ofstr.open(depth_error_filename.c_str());
    ofstr << "P3" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    int index = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x, ++index) {
        Vector3f rgb;
        if (depth_errors[index] == kInvalid) {
          rgb = Vector3f(0, 0, 0);
        } else {
          const double gray = min(1.0, depth_errors[index] / (depth_unit * kMaxDepthError));
          if (gray < 0.5) {
            rgb[1] = 255.0 * 2.0 * gray;
            rgb[2] = 255.0 - rgb[1];
            rgb[0] = 0.0;
          } else {
            rgb[2] = 0;
            rgb[0] = 255.0 * 2.0 * (gray - 0.5);
            rgb[1] = 255.0 - rgb[0];
          }
        }
        Vector3f color = small_panorama.GetRGB(Vector2d(x, y));
        swap(color[0], color[2]);
        color = color * alpha + rgb * (1.0 - alpha);
        for (int i = 0; i < 3; ++i)
          ofstr << static_cast<int>(round(color[i])) << ' ';
      }
    }
    ofstr.close();
  }

  {
    const double kMaxNormalError = 4.0;
    ofstream ofstr;
    ofstr.open(normal_error_filename.c_str());
    ofstr << "P3" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    int index = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x, ++index) {
        Vector3f rgb;
        if (normal_errors[index] == kInvalid) {
          rgb = Vector3f(0, 0, 0);
        } else {
          const double gray = min(1.0, normal_errors[index] / kMaxNormalError);
          if (gray < 0.5) {
            rgb[1] = 255.0 * 2.0 * gray;
            rgb[2] = 255.0 - rgb[1];
            rgb[0] = 0.0;
          } else {
            rgb[2] = 0;
            rgb[0] = 255.0 * 2.0 * (gray - 0.5);
            rgb[1] = 255.0 - rgb[0];
          }
        }
        Vector3f color = small_panorama.GetRGB(Vector2d(x, y));
        swap(color[0], color[2]);
        color = color * alpha + rgb * (1.0 - alpha);
        for (int i = 0; i < 3; ++i)
          ofstr << static_cast<int>(round(color[i])) << ' ';
      }
    }
    ofstr.close();
  }
}

void VisualizeResults(const FileIO& file_io, const string prefix,
                      const std::vector<PointCloud>& input_point_clouds,
                      const std::vector<std::vector<RasterizedGeometry> >& rasterized_geometries,
                      const std::vector<Panorama>& panoramas,
                      const double invalid_depth,
                      const double depth_unit) {
  for (int p = 0; p < rasterized_geometries.size(); ++p) {
    {
      char depth_filename[1024];
      sprintf(depth_filename,
              "%s/images/%03d_depth_%s.ppm",
              file_io.GetEvaluationDirectory().c_str(), p, prefix.c_str());
      WriteDepthmap(panoramas[p], rasterized_geometries[p], invalid_depth, depth_filename);
    }
    {
      char depth_error_filename[1024], normal_error_filename[1024];
      sprintf(depth_error_filename,
              "%s/images/%03d_depth_error_%s.ppm",
              file_io.GetEvaluationDirectory().c_str(), p, prefix.c_str());
      sprintf(normal_error_filename,
              "%s/images/%03d_normal_error_%s.ppm",
              file_io.GetEvaluationDirectory().c_str(), p, prefix.c_str());
      WriteDepthErrormap(input_point_clouds[p], rasterized_geometries[p], panoramas[p],
                         invalid_depth, depth_unit, depth_error_filename, normal_error_filename);
    }
  }

    /*
  for (int p = 0; p < rasterized_geometries.size(); ++p) {
    const Panorama& panorama = panoramas[p];
    const int width  = panorama.DepthWidth();
    const int height = panorama.DepthHeight();
    
    bool first = true;
    double max_depth, min_depth;
    for (int i = 0; i < rasterized_geometries[p].size(); ++i) {
      if (rasterized_geometries[p][i].depth != invalid_depth) {
        if (first) {
          max_depth = min_depth = rasterized_geometries[p][i].depth;
          first = false;
        } else {
          max_depth = max(max_depth, rasterized_geometries[p][i].depth);
          min_depth = min(min_depth, rasterized_geometries[p][i].depth);
        }
      }
    }
    
    ofstream ofstr;
    char buffer[1024];
    sprintf(buffer, "%s/%03d_%s.ppm", file_io.GetEvaluationDirectory().c_str(), p, prefix.c_str());
    ofstr.open(buffer);
    ofstr << "P3" << endl
          << width << ' ' << height << endl
          << 255 << endl;
    for (int i = 0; i < rasterized_geometries[p].size(); ++i) {
      if (rasterized_geometries[p][i].depth == invalid_depth) {
        ofstr << "255 0 0 ";
      } else {
        const int itmp = (int)(255 * (rasterized_geometries[p][i].depth - min_depth) / (max_depth - min_depth));
        ofstr << itmp << ' ' << itmp << ' ' << itmp << ' ';
      }
    }
    ofstr.close();
  }
    */
}

void Initialize(const FileIO& file_io, Floorplan* floorplan, IndoorPolygon* indoor_polygon) {
  string floorplan_file;
  if (FLAGS_floorplan_file == "") {
    floorplan_file = file_io.GetFloorplan();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", file_io.GetDataDirectory().c_str(), FLAGS_floorplan_file.c_str());
    floorplan_file = buffer;
  }

  string indoor_polygon_file;
  if (FLAGS_indoor_polygon_file == "") {
    indoor_polygon_file = file_io.GetIndoorPolygonWithCeiling();

    ifstream ifstr;
    ifstr.open(indoor_polygon_file);
    if (!ifstr.is_open())
      indoor_polygon_file = file_io.GetIndoorPolygon();
  } else {
    char buffer[1024];
    sprintf(buffer, "%s/%s", file_io.GetDataDirectory().c_str(), FLAGS_indoor_polygon_file.c_str());
    indoor_polygon_file = buffer;
  }

  *floorplan      = Floorplan(floorplan_file);

  ifstream ifstr;
  ifstr.open(indoor_polygon_file);
  if (ifstr.is_open()) {
    ifstr.close();
    *indoor_polygon = IndoorPolygon(indoor_polygon_file);
  }
}
  
}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
#ifdef __APPLE__
  google::ParseCommandLineFlags(&argc, &argv, true);
#else
  gflags::ParseCommandLineFlags(&argc, &argv, true);
#endif

  FileIO file_io(argv[1]);
  Floorplan floorplan;
  IndoorPolygon indoor_polygon;
  Initialize(file_io, &floorplan, &indoor_polygon);

  vector<Panorama> panoramas;
  ReadPanoramas(file_io, &panoramas);
  // Adjust the center of panorama to the center of the laser scanner.
  for (int p = 0; p < panoramas.size(); ++p) {
    panoramas[p].AdjustCenter(GetLaserCenter(file_io, p));
  }  

  vector<PointCloud> input_point_clouds, object_point_clouds;
  ReadPointClouds(file_io, &input_point_clouds);
  ReadObjectPointClouds(file_io, floorplan.GetNumRooms(), &object_point_clouds);

  // Accuracy and completeness.
  const RasterizedGeometry kInitial(numeric_limits<double>::max(), Vector3d(0, 0, 0), kHole);
  std::vector<std::vector<RasterizedGeometry> > rasterized_geometries;

  double depth_unit = 0.0;
  for (int p = 0; p < panoramas.size(); ++p)
    depth_unit += panoramas[p].GetAverageDistance();
  depth_unit /= panoramas.size();

  //----------------------------------------------------------------------
  if (FLAGS_evaluate_all || FLAGS_evaluate_floorplan) {
    // Floorplan only.
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeFloorplan(floorplan, panoramas, &rasterized_geometries);
    VisualizeResults(file_io, "floorplan", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
    ReportErrors(file_io, "floorplan", input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
  }
  
  // Indoor polygon only.
  if (FLAGS_evaluate_all || FLAGS_evaluate_indoor_polygon) {
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeIndoorPolygon(indoor_polygon, panoramas, &rasterized_geometries);
    VisualizeResults(file_io, "indoor_polygon", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
    ReportErrors(file_io, "indoor_polygon", input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
  }

  if (FLAGS_evaluate_all || FLAGS_evaluate_indoor_polygon_and_object_point_clouds) {
    Initialize(panoramas, kInitial, &rasterized_geometries);
    RasterizeIndoorPolygon(indoor_polygon, panoramas, &rasterized_geometries);
    RasterizeObjectPointClouds(object_point_clouds, panoramas, &rasterized_geometries);
    VisualizeResults(file_io, "indoor_polygon_and_object_point_clouds", input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
    ReportErrors(file_io, "indoor_polygon_and_object_point_clouds", input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
  }

  if (FLAGS_evaluate_all || FLAGS_evaluate_poisson_mesh) {
    {
      const vector<string> filenames = file_io.GetPoissonMeshes();
      for (int i = 1; i < filenames.size(); ++i) {
	Mesh poisson_mesh;
	if (!ReadMesh(filenames[i], &poisson_mesh))
          continue;
	Initialize(panoramas, kInitial, &rasterized_geometries);
	RasterizeMesh(poisson_mesh, panoramas, &rasterized_geometries);
	
	char buffer[1024];
	sprintf(buffer, "poisson%d", i);
	VisualizeResults(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
	ReportErrors(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
      }
    }
    {
      const vector<string> filenames = file_io.GetFilteredPoissonMeshes();
      for (int i = 1; i < filenames.size(); ++i) {
	Mesh poisson_mesh;
	if (!ReadMesh(filenames[i], &poisson_mesh))
          continue;
	Initialize(panoramas, kInitial, &rasterized_geometries);
	RasterizeMesh(poisson_mesh, panoramas, &rasterized_geometries);
	
	char buffer[1024];
	sprintf(buffer, "poisson_filtered%d", i);
	VisualizeResults(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
	ReportErrors(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
      }
    }
  }

  if (FLAGS_evaluate_all || FLAGS_evaluate_vgcut_mesh) {
    {
      const vector<string> filenames = file_io.GetVgcutMeshes();
      for (int i = 0; i < filenames.size(); ++i) {
	Mesh vgcut_mesh;
	if (!ReadMesh(filenames[i], &vgcut_mesh))
          continue;
	Initialize(panoramas, kInitial, &rasterized_geometries);
	RasterizeMesh(vgcut_mesh, panoramas, &rasterized_geometries);
	
	char buffer[1024];
	sprintf(buffer, "vgcut%d", i);
	VisualizeResults(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
	ReportErrors(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
      }
    }
    {
      const vector<string> filenames = file_io.GetFilteredVgcutMeshes();
      for (int i = 0; i < filenames.size(); ++i) {
	Mesh vgcut_mesh;
	if (!ReadMesh(filenames[i], &vgcut_mesh))
          continue;
	Initialize(panoramas, kInitial, &rasterized_geometries);
	RasterizeMesh(vgcut_mesh, panoramas, &rasterized_geometries);
	
	char buffer[1024];
	sprintf(buffer, "vgcut_filtered%d", i);
	VisualizeResults(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial.depth, depth_unit);
	ReportErrors(file_io, buffer, input_point_clouds, rasterized_geometries, panoramas, kInitial, depth_unit);
      }
    }
  }
  
  return 0;
}
