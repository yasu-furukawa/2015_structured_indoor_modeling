#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <gflags/gflags.h>


#include "../calibration/file_io.h"
#include "../floorplan/floorplan.h"
#include "../floorplan/panorama.h"
#include "generate_texture.h"

DEFINE_int32(num_panoramas, 1, "Number of panorama images.");
DEFINE_int32(num_pyramid_levels, 3, "Num pyramid levels.");
DEFINE_double(texel_size_rescale, 1.0, "Less than 1 to increase resolution.");
DEFINE_int32(max_texture_size_per_patch, 1024, "Maximum texture size for each patch.");

using namespace Eigen;
using namespace std;
using namespace texture;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " data_directory" << endl;
    return 1;
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InitGoogleLogging(argv[0]);

  // Read data from the directory.
  file_io::FileIO file_io(argv[1]);
  vector<vector<Panorama> > panoramas;
  {
    ReadPanoramas(file_io,
                  FLAGS_num_panoramas,
                  FLAGS_num_pyramid_levels,
                  &panoramas);
  }
  vector<Matrix4d> panorama_to_globals;
  {
    ReadPanoramaToGlobals(file_io, FLAGS_num_panoramas, &panorama_to_globals);
   }
  Floorplan floorplan;
  {
    const string filename = file_io.GetFloorplan();
    ifstream ifstr;
    ifstr.open(filename.c_str());
    ifstr >> floorplan;
    ifstr.close();
  }
  vector<Matrix4d> global_to_panoramas;
  {
    Invert(panorama_to_globals, &global_to_panoramas);
  }

  // Unit for a texel.
  double texel_size;
  ComputeTexelSize(panoramas);
  texel_size *= FLAGS_texel_size_rescale;
  
  // For each wall rectangle, floor, and ceiling,
  // 0. Identify visible panoramas.
  // 1. Grab texture
  // 2. Stitch
  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
    for (int wall = 0; wall < floorplan.GetNumWalls(room); ++wall) {
      const int next_wall = (wall + 1) % floorplan.GetNumWalls(room);
      Patch patch;
      patch.vertices[0] = Vector3d(floorplan.GetRoomVertexLocal(room, wall)[0],
                                   floorplan.GetRoomVertexLocal(room, wall)[1],
                                   floorplan.GetCeilingHeight(room));
      patch.vertices[1] = Vector3d(floorplan.GetRoomVertexLocal(room, next_wall)[0],
                                   floorplan.GetRoomVertexLocal(room, next_wall)[1],
                                   floorplan.GetCeilingHeight(room));
      patch.vertices[2] = Vector3d(floorplan.GetRoomVertexLocal(room, next_wall)[0],
                                   floorplan.GetRoomVertexLocal(room, next_wall)[1],
                                   floorplan.GetFloorHeight(room));
      patch.vertices[3] = Vector3d(floorplan.GetRoomVertexLocal(room, wall)[0],
                                   floorplan.GetRoomVertexLocal(room, wall)[1],
                                   floorplan.GetFloorHeight(room));

      patch.x_axis = (patch.vertices[1] - patch.vertices[0]).normalized();
      patch.y_axis = (patch.vertices[3] - patch.vertices[0]).normalized();
      
      // Identify visible panoramas.
      vector<pair<double, int> > visible_panoramas_weights;
      FindVisiblePanoramas(panoramas, global_to_panoramas, patch, &visible_panoramas_weights);

      // Keep the best one only.
      const int best_panorama = visible_panoramas_weights[0].second;

      SetTextureSize(texel_size, FLAGS_max_texture_size_per_patch, &patch);
      // Grab texture.
      GrabTexture(panoramas[best_panorama], &patch);


      
    }      
  }
}
