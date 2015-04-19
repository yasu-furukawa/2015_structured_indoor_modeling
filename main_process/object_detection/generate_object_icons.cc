#include <fstream>
#include <iostream>
#include "../../base/indoor_polygon.h"
#include "polygon_triangulation2.h"
#include "generate_object_icons.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const ObjectId kInitialObject  = make_pair<int, int>(-1, -1);
const ObjectId kNoObject = make_pair<int, int>(-2, -2);

namespace {

ObjectId FindObject(const std::vector<Panorama>& panoramas,
                    const std::vector<std::vector<ObjectId> >& object_id_maps,
                    const Detection& detection,
                    const double area_threshold) {
  const Panorama& panorama = panoramas[detection.panorama];
  const vector<ObjectId>& object_id_map = object_id_maps[detection.panorama];

  const int width  = panorama.DepthWidth();
  const int height = panorama.DepthHeight();

  const int xs[2] = { static_cast<int>(round(width * detection.us[0])),
                      static_cast<int>(round(width * detection.us[1])) };
  const int ys[2] = { static_cast<int>(round(height * detection.vs[0])),
                      static_cast<int>(round(height * detection.vs[1])) };

  map<ObjectId, int> counts;
  int total_count = 0;
  for (int y = ys[0]; y < min(height, ys[1]); ++y) {
    for (int x = xs[0]; x < xs[1]; ++x) {
      ++total_count;
      const int xtmp = x % width;
      const int index = y * width + xtmp;
      if (object_id_map[index] == kInitialObject)
        continue;
      counts[object_id_map[index]] += 1;
    }
  }

  // Find the object id with the most count.
  int best_count = 0;
  ObjectId best_object_id;
  for (const auto& count : counts) {
    if (count.second > best_count) {
      best_count = count.second;
      best_object_id = count.first;
    }
  }

  if (best_count > total_count * area_threshold)
    return best_object_id;
  else
    return kNoObject;
}

}  // namespace
  
void RasterizeObjectIds(const std::vector<Panorama>& panoramas,
                        const std::vector<PointCloud>& object_point_clouds,
                        std::vector<std::vector<ObjectId> >* object_id_maps) {
  const double kThresholdRatio = 0.05;
  
  const int num_panoramas = (int)panoramas.size();
  const int num_rooms = (int)object_point_clouds.size();
  object_id_maps->clear();
  object_id_maps->resize(num_panoramas);

  cerr << "RasterizeObjectIds:" << flush;
  for (int p = 0; p < num_panoramas; ++p) {
    cerr << "." << flush;
    const Panorama& panorama = panoramas[p];
    const double visibility_threshold = panorama.GetAverageDistance() * kThresholdRatio;

    const int width  = panorama.DepthWidth();
    const int height = panorama.DepthHeight();
    object_id_maps->at(p).resize(width * height, kInitialObject);

    for (int room = 0; room < num_rooms; ++room) {
      const PointCloud& point_cloud = object_point_clouds[room];

      for (int q = 0; q < point_cloud.GetNumPoints(); ++q) {
        const Point& point = point_cloud.GetPoint(q);
        if (point.object_id == -1) {
          cerr << "No object id assigned to a point." << endl;
          exit (1);
        }
        const ObjectId object_id = pair<int, int>(room, point.object_id);

        const Vector2d depth_pixel = panorama.ProjectToDepth(point.position);
        const double depth = panorama.GetDepth(depth_pixel);
        const double distance = (point.position - panorama.GetCenter()).norm();

        // Invisible.
        if (distance > depth + visibility_threshold)
          continue;

        const int u = static_cast<int>(round(depth_pixel[0]));
        const int v = static_cast<int>(round(depth_pixel[1]));
        object_id_maps->at(p)[v * width + u] = object_id;
      }
    }

    /*
    //?????
    {
      map<ObjectId, Vector3i> color_table;
      char buffer[1024];
      sprintf(buffer, "%03d.ppm", p);
      ofstream ofstr;
      ofstr.open(buffer);
      ofstr << "P3" << endl
            << width << ' ' << height << endl
            << 255 << endl;
      int index = 0;
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x, ++index) {
          if (object_id_maps->at(p)[index] == kInitialObject) {
            ofstr << "255 255 255 ";
          } else {
            if (color_table.find(object_id_maps->at(p)[index]) == color_table.end()) {
              Vector3i color(rand() % 255, rand() % 255, rand() % 255);
              color_table[object_id_maps->at(p)[index]] = color;

              if (object_id_maps->at(p)[index].second == 42)
                color_table[object_id_maps->at(p)[index]] = Vector3i(255, 0, 0);
              
            }
            ofstr << color_table[object_id_maps->at(p)[index]][0] << ' '
                  << color_table[object_id_maps->at(p)[index]][1] << ' '
                  << color_table[object_id_maps->at(p)[index]][2] << ' ';
          }
        }
      }
      ofstr.close();
    }
    */

  }
  cerr << endl;
}

void AssociateObjectId(const std::vector<Panorama>& panoramas,
                       const std::vector<Detection>& detections,
                       const std::vector<std::vector<ObjectId> >& object_id_maps,
                       const double score_threshold,
                       const double area_threshold,
                       std::map<ObjectId, int>* object_to_detection) {
  // For each detection, corresponding object id.
  object_to_detection->clear();
  vector<bool> detection_used((int)detections.size(), false);
  //----------------------------------------------------------------------
  // Greedy assignment.
  while (true) {
    // Pick the detection whose value is kInitial and has the highest score.
    const int kInvalid = -1;
    int best_detection_index = kInvalid;
    double best_detection_score = -100.0;

    for (int index = 0; index < (int)detections.size(); ++index) {
      const Detection& detection = detections[index];
      if (detection_used[index])
        continue;
      if (detection.score < score_threshold)
        continue;
      
      if (detection.score > best_detection_score) {
        best_detection_score = detection.score;
        best_detection_index = index;
      }
    }
    if (best_detection_index == kInvalid)
      break;

    detection_used[best_detection_index] = true;
    // Find the object inside the detection.
    const ObjectId best_object =
      FindObject(panoramas, object_id_maps, detections[best_detection_index], area_threshold);

    if (best_object == kNoObject)
      continue;

    // If best_object has already an associated detection ignore.
    if (object_to_detection->find(best_object) != object_to_detection->end())
        continue;

    (*object_to_detection)[best_object] = best_detection_index;
  }
}

void AddIconInformationToDetections(const IndoorPolygon& indoor_polygon,
                                    const std::vector<PointCloud>& object_point_clouds,
                                    const std::map<ObjectId, int>& object_to_detection,
                                    std::vector<Detection>* detections) {
    const int num_room = object_point_clouds.size();
    vector<vector<bool> >isAdded(num_room);
    for(int roomid=0; roomid<isAdded.size(); roomid++){
	isAdded[roomid].resize(object_point_clouds[roomid].GetNumObjects());
	for(int objid=0; objid<isAdded[roomid].size(); objid++)
	    isAdded[roomid][objid] = false;
    }
    
  for (const auto& item : object_to_detection) {
    const ObjectId& object_id = item.first;
    Detection& detection = detections->at(item.second);
    detection.room = object_id.first;
    detection.object = object_id.second;

    isAdded[detection.room][detection.object] = true;

    vector<Point> points;
    vector<Vector3d>manhattanpoints;
    object_point_clouds[detection.room].GetObjectPoints(detection.object, points);

    vector<double> histograms[3];
    for (const auto& point : points) {
      const Vector3d& manhattan = indoor_polygon.GlobalToManhattan(point.position);
      manhattanpoints.push_back(manhattan);
      for (int a = 0; a < 3; ++a) {
        histograms[a].push_back(manhattan[a]);
      }
    }

    // 5 percentile and 95 percentile.
    for (int a = 0; a < 3; ++a) {
      vector<double>::iterator lower_ite =
        histograms[a].begin() + static_cast<int>(round(histograms[a].size() * 0.05));
      vector<double>::iterator upper_ite =
        histograms[a].begin() + static_cast<int>(round(histograms[a].size() * 0.95));
      
      nth_element(histograms[a].begin(), lower_ite, histograms[a].end());
      detection.ranges[a][0] = *lower_ite;
      nth_element(histograms[a].begin(), upper_ite, histograms[a].end());
      detection.ranges[a][1] = *upper_ite;
    }
    //compute polygon
    ComputeObjectPolygon(manhattanpoints, detection);
  }

  //Add non-detected objects
  const double min_area = 1e5;
  for(int roomid=0; roomid<num_room; ++roomid){
      const PointCloud& room_cloud = object_point_clouds[roomid];
      for(int objid=0; objid<room_cloud.GetNumObjects(); ++objid){
	  if(isAdded[roomid][objid])
	      continue;

	  Detection detection;
	  detection.room = roomid;
	  detection.object = objid;
	  detection.names.resize(1);
	  detection.names[0] = "unknown";

	  vector<Point>objpt;
	  vector<Vector3d>manhattanpoints;
	  room_cloud.GetObjectPoints(objid, objpt);
	  
	  vector<double> histograms[3];
	  for (const auto& point : objpt) {
	      Vector3d manhattan = indoor_polygon.GlobalToManhattan(point.position);
	      manhattanpoints.push_back(manhattan);
	      for (int a = 0; a < 3; ++a) {
		  histograms[a].push_back(manhattan[a]);
	      }
	  }
	  // 5 percentile and 95 percentile.
	  for (int a = 0; a < 3; ++a) {
	      vector<double>::iterator lower_ite =
		  histograms[a].begin() + static_cast<int>(round(histograms[a].size() * 0.05));
	      vector<double>::iterator upper_ite =
		  histograms[a].begin() + static_cast<int>(round(histograms[a].size() * 0.95));
      
	      nth_element(histograms[a].begin(), lower_ite, histograms[a].end());
	      detection.ranges[a][0] = *lower_ite;
	      nth_element(histograms[a].begin(), upper_ite, histograms[a].end());
	      detection.ranges[a][1] = *upper_ite;
	  }
	  
	  const double area_on_floorplan = (detection.ranges[0][1] - detection.ranges[0][0]) * (detection.ranges[1][1] - detection.ranges[1][0]);
	  if(area_on_floorplan >= min_area){
	       ComputeObjectPolygon(manhattanpoints, detection);
	       detections->push_back(detection);
	  }

      }
  }
  
}


    void ComputeObjectPolygon(const vector<Vector3d>& manhattan,
			      Detection &detection){
	static const double grid_size = 40.0;
	//asslocate grid
	const int size_x = ceil((detection.ranges[0][1] - detection.ranges[0][0])/grid_size);
	const int size_y = ceil((detection.ranges[1][1] - detection.ranges[1][0])/grid_size);
	const int margin_begin = 5;
	const int margin_end = 5;
	vector<vector<double> >grid(size_y+margin_end+margin_begin + 5);
	for(auto& v:grid){
	    v.resize(size_x+margin_end+margin_begin + 5);
	    for(auto& vv:v)
		vv = 0.0;
	}
	for(const auto&point: manhattan){
	    const double curx = (point[0] - detection.ranges[0][0]) / grid_size;
	    const double cury = (point[1] - detection.ranges[1][0]) / grid_size;
	    if(curx >=0 && floor(curx)<size_x &&
	       cury >=0 && floor(cury)<size_y){
		grid[floor(cury)+margin_begin][floor(curx)+margin_begin] += 1.0;
	    }
	}

	const double isovalue = 0.5;
	const int dialate_iter = 3;
	//dialate
	for(int iter=0; iter<dialate_iter; ++iter){
	     vector<vector<double> >grid_copy = grid;
	     for(int y=margin_begin; y<size_y+margin_begin; ++y){
		  for(int x=margin_begin; x<size_x+margin_begin; ++x){
		       for(int dy=-1; dy<=1; ++dy){
			    for(int dx=-1; dx<=1; ++dx){
				 if(dx==0 && dy==0)
				      continue;
				 if(grid_copy[y+dy][x+dx] >= isovalue){
				      grid[y][x] = isovalue + 1.0;
				      break;
				 }

			    }
		       }
		  }
	     }
	}

	vector<Vector2i>linelist;
	MarchingCube(grid, detection.vlist, linelist, isovalue);
	SortPolygon(detection.vlist, linelist);
//	Smoothing(detection.vlist, 1);
	Simplification(detection.vlist, 40, 0.01);

	//triangulation
	Cgal::Triangulate(detection.vlist, &detection.elist);
	
	for(auto &v :detection.vlist){
	    v[0] = (v[0]-margin_begin)*grid_size + detection.ranges[0][0];
	    v[1] = (v[1]-margin_begin)*grid_size + detection.ranges[1][0];
	}
	
    }

     void MarchingCube(std::vector<std::vector<double> >&grid,
		       std::vector<Vector2d>&vlist,
		       std::vector<Vector2i>&elist,
		       const double isovalue){
	  if(grid.size() == 0)
	       return;
	  cout<<"Marching cube..."<<endl<<flush;
	  const int size_y = grid.size();
	  const int size_x = grid[0].size();
	  vector<vector<Vector2i> >ptindex(size_y);
	  {
	       for(auto& v:ptindex){
		    v.resize(size_x);
		    for(auto&vv: v)
			 vv = Vector2i(-1,-1);;
	       }
	  }

	  for(int y=0;y<size_y; ++y){
	       for(int x=0;x<size_x;++x){
		    if(grid[y][x] == isovalue)
			 grid[y][x] -= 0.2;
	       }
	  }

	  //initialize shape table
	  for(int y=0; y<size_y-1; ++y){
	       for(int x=0; x<size_x-1; ++x){
		    unsigned char curshape = 0;
		    if(grid[y][x] > isovalue)
			 curshape = curshape | 8;
		    if(grid[y][x+1] > isovalue)
			 curshape = curshape | 4;
		    if(grid[y+1][x+1] > isovalue)
			 curshape = curshape | 2;
		    if(grid[y+1][x] > isovalue)
			 curshape = curshape | 1;
		    if(curshape == 8 || curshape == 7){
			Vector2i curedge;
			Vector2d pt1(x+(isovalue-grid[y][x])/(grid[y][x+1]-grid[y][x]),y);
			Vector2d pt2(x, y+(isovalue-grid[y][x]) / (grid[y+1][x] - grid[y][x]));
			if(ptindex[y][x][0] != -1)
			    curedge[0] = ptindex[y][x][0];
			else{
			    vlist.push_back(pt1);
			    ptindex[y][x][0] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}

			if(ptindex[y][x][1] != -1)
			    curedge[1] = ptindex[y][x][1];
			else{
			    vlist.push_back(pt2);
			    ptindex[y][x][1] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }
		    if(curshape == 11 || curshape == 4){
			Vector2i curedge;
			Vector2d pt1(x+(isovalue-grid[y][x])/(grid[y][x+1]-grid[y][x]),y);
			Vector2d pt2(x+1, y+(isovalue-grid[y][x+1]) / (grid[y+1][x+1] - grid[y][x+1]));
			if(ptindex[y][x][0] != -1)
			    curedge[0] = ptindex[y][x][0];
			else{
			    vlist.push_back(pt1);
			    ptindex[y][x][0] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}
			if(ptindex[y][x+1][1] != -1)
			    curedge[1] = ptindex[y][x+1][1];
			else{
			    vlist.push_back(pt2);
			    ptindex[y][x+1][1] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }
		    
		    if(curshape == 2 || curshape == 13){
			Vector2i curedge;
			Vector2d pt1(x+(isovalue-grid[y+1][x])/(grid[y+1][x+1]-grid[y+1][x]),y+1);
			Vector2d pt2(x+1, y+(isovalue-grid[y][x+1]) / (grid[y+1][x+1] - grid[y][x+1]));
			if(ptindex[y+1][x][0] != -1)
			    curedge[0] = ptindex[y+1][x][0];
			else{
			    vlist.push_back(pt1);
			    ptindex[y+1][x][0] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}
			if(ptindex[y][x+1][1] != -1)
			    curedge[1] = ptindex[y][x+1][1];
			else{
			    vlist.push_back(pt2);
			    ptindex[y][x+1][1] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }
		    
		    if(curshape == 1 || curshape == 14){
			Vector2i curedge;
			Vector2d pt1(x+(isovalue-grid[y+1][x])/(grid[y+1][x+1]-grid[y+1][x]),y+1);
			Vector2d pt2(x, y+(isovalue-grid[y][x]) / (grid[y+1][x] - grid[y][x]));
			if(ptindex[y+1][x][0] != -1)
			    curedge[0] = ptindex[y+1][x][0];
			else{
			    vlist.push_back(pt1);
			    ptindex[y+1][x][0] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}
			if(ptindex[y][x][1] != -1)
			    curedge[1] = ptindex[y][x][1];
			else{
			    vlist.push_back(pt2);
			    ptindex[y][x][1] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }
		    
		    if(curshape == 9 || curshape == 6){
			Vector2i curedge;
			Vector2d pt1(x+(isovalue-grid[y][x])/(grid[y][x+1]-grid[y][x]),y);
			Vector2d pt2(x+(isovalue-grid[y+1][x])/(grid[y+1][x+1]-grid[y+1][x]),y+1);
			if(ptindex[y][x][0] != -1)
			    curedge[0] = ptindex[y][x][0];
			else{
			    vlist.push_back(pt1);
			    ptindex[y][x][0] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}
			if(ptindex[y+1][x][0] != -1)
			    curedge[1] = ptindex[y+1][x][0];
			else{

			    vlist.push_back(pt2);
			    ptindex[y+1][x][0] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }
		    
		    if(curshape == 3 || curshape == 12){
			Vector2i curedge;
			Vector2d pt1(x, y+(isovalue-grid[y][x]) / (grid[y+1][x] - grid[y][x]));
			Vector2d pt2(x+1, y+(isovalue-grid[y][x+1]) / (grid[y+1][x+1] - grid[y][x+1]));
			if(ptindex[y][x][1] != -1)
			    curedge[0] = ptindex[y][x][1];
			else{

			    vlist.push_back(pt1);
			    ptindex[y][x][1] = vlist.size() - 1;
			    curedge[0] = vlist.size() - 1;
			}
			if(ptindex[y][x+1][1] != -1)
			    curedge[1] = ptindex[y][x+1][1];
			else{

			    vlist.push_back(pt2);
			    ptindex[y][x+1][1] = vlist.size() - 1;
			    curedge[1] = vlist.size() - 1;
			}
			elist.push_back(curedge);
			continue;
		    }

		    
		    if(curshape == 10 || curshape == 5){
			Vector2i curedge1, curedge2;
			 Vector2d pt1(x+(isovalue-grid[y][x])/(grid[y][x+1]-grid[y][x]),y);
			 Vector2d pt2(x+1, y+(isovalue-grid[y][x+1]) / (grid[y+1][x+1] - grid[y][x+1]));
			 Vector2d pt3(x+(isovalue-grid[y+1][x])/(grid[y+1][x+1]-grid[y+1][x]),y+1);
			 Vector2d pt4(x, y+(isovalue-grid[y][x]) / (grid[y+1][x] - grid[y][x]));
			 if(curshape == 10){
			     if(ptindex[y][x][0] != -1)
				 curedge1[0] = ptindex[y][x][0];
			     else
			     {
				 vlist.push_back(pt1);
				 curedge1[0] = vlist.size() - 1;
				 ptindex[y][x][0] = curedge1[0];
			     }
			     if(ptindex[y][x][1] != -1)
				 curedge1[1] = ptindex[y][x][1];
			     else{
				 vlist.push_back(pt4);
				 curedge1[1] = vlist.size() - 1;
				 ptindex[y][x][1] = curedge1[1];
			     }

			     if(ptindex[y][x+1][1] != -1)
				 curedge2[0] = ptindex[y][x+1][1];
			     else{
				 vlist.push_back(pt2);
				 curedge2[0] = vlist.size() - 1;
				 ptindex[y][x+1][1] = curedge2[0];
			     }
			     if(ptindex[y+1][x][0] != -1)
				 curedge2[1] = ptindex[y+1][x][0];
			     else{
				 vlist.push_back(pt3);
				 curedge2[1] = vlist.size() - 1;
				 ptindex[y+1][x][0] = curedge2[1];
			     }
			 }
			 
			 if(curshape == 5){
			     if(ptindex[y][x][0] != -1)
				 curedge1[0] = ptindex[y][x][0];
			     else{
				 vlist.push_back(pt1);
				 curedge1[0] = vlist.size() - 1;
				 ptindex[y][x][0] = curedge1[0];
			     }
			     if(ptindex[y][x+1][1] != -1)
				 curedge1[1] = ptindex[y][x+1][1];
			     else{
				 vlist.push_back(pt2);
				 curedge1[1] = vlist.size() - 1;
				 ptindex[y][x+1][1] = curedge1[1];
			     }
			     if(ptindex[y][x][1] != -1)
				 curedge2[0] = ptindex[y][x][1];
			     else{
				 vlist.push_back(pt4);
				 curedge2[0] = vlist.size() - 1;
				 ptindex[y][x][1] = curedge2[0];
			     }
			     if(ptindex[y+1][x][0] != -1)
				 curedge2[1] = ptindex[y+1][x][0];
			     else{
				 vlist.push_back(pt3);
				 curedge2[1] = vlist.size() - 1;
				 ptindex[y+1][x][0] = curedge2[1];
			     }
			 }			 
		    }
	       }
	  }
     }

    bool SanityCheck(const vector<Vector2i>&elist){
	vector<int> sanity(10000);
	for(auto &v: sanity)
	    v = 0;
	for(const auto& edge: elist){
	    sanity[edge[0]] += 1;
	    sanity[edge[1]] += 1;
	}
	bool iscorrect = true;
	for(const auto&v: sanity)
	    if(v % 2 == 1)
		iscorrect = false;
	return iscorrect;
    }

     void SortPolygon(vector<Vector2d>&vlist, vector<Vector2i>&elist){
	  if(elist.size() == 0)
	       return;
	  //sanity check
	  if(!SanityCheck(elist)){
//	      cout<<"Fore-check: incomplete loop!"<<endl;
	      return;
	  }
//	  cout<<"Fore-check passed"<<endl;

//	  cout<<"Sort polygon..."<<endl<<flush;
	  vector<vector<Vector2i> >esorted;
	  vector<bool>issorted(elist.size());
	  for(int i=0; i<elist.size(); ++i)
	       issorted[i] = false;
	  while(1){
	       //find the first unsorted edge
	       int first;
	       for(first=0; first<elist.size(); first++){
		    if(issorted[first] == false)
			 break;
	       }
	       if(first == elist.size())
		    break;
	       vector<Vector2i>curelist;
	       issorted[first] = true;
	       curelist.push_back(elist[first]);
	       while(1){
		    bool isadded = false;
		    for(int eid=0; eid<elist.size(); ++eid){
			 if(issorted[eid])
			      continue;
			 if(elist[eid][0] == curelist.back()[1]){
			      isadded = true;
			      curelist.push_back(elist[eid]);
			      issorted[eid] = true;
			 }else if(elist[eid][1] == curelist.back()[1]){
			      isadded = true;
			      curelist.push_back(Vector2i(elist[eid][1],elist[eid][0]));
			      issorted[eid] = true;
			 }
		    }
		    if(!isadded)
			 break;
	       }
	       if(esorted.size() == 0)
		   esorted.push_back(curelist);
	       else if(curelist.size() > esorted.back().size())
		   esorted.push_back(curelist);
	  }
	  elist.swap(esorted.back());
	  if(!SanityCheck(elist)){
//	      cout<<"Post-check: incomplete loop!"<<endl;
	      return;
	  }
	  vector<Vector2d>reslist;
	  int lastv = -1;
	  for(const auto&edge: elist){
	       if(edge[0] != lastv){
		    reslist.push_back(vlist[edge[0]]);
		    lastv = edge[0];
	       }
	       if(edge[1] != lastv){
		    reslist.push_back(vlist[edge[1]]);
		    lastv = edge[1];
	       }
	  }
	  reslist.pop_back();
	  vlist.swap(reslist);
     }

     void Smoothing(vector<Vector2d>& vlist, const int iteration, const double lambda){
	  if(lambda == 0)
	       return;
	  for(int iter = 0; iter<iteration * 2; iter++){
	       double curlambda;
	       if(iter % 2 == 0)
		    curlambda = lambda;
	       else
		    curlambda = 1 / (0.1 - 1/lambda);

	       vector<Vector2d>ori = vlist;
	       vlist[0] = curlambda * ori[0] + (1-curlambda) * (ori.back() + ori[2]) / 2;
	       for(int vid=1; vid<ori.size() - 1; ++vid){
		    vlist[vid] = curlambda * ori[vid] + (1 - curlambda) * (ori[vid-1]+ori[vid+1])/2;
	       }
//	       vlist.back() = curlambda * ori.back() + (1-lambda) *(ori[0] + ori[ori.size()-2])/2;
	       
	  }
     }

     void Simplification(vector<Vector2d>&vlist, const int target, const double margin){
	  cout<<"Simplification..."<<endl;
	  //remove colinear points
	  vector<Vector2d>reslist;
	  reslist.push_back(vlist[0]);
	  for(int vid=1; vid<vlist.size()-1; ++vid){
	       Vector2d line = vlist[vid-1] - vlist[vid+1];
	       Vector2d n(-1*line[1], line[0]);
	       n.normalize();
	       double dis = abs((vlist[vid]-vlist[vid-1]).dot(n));
	       if(dis >= margin)
		    reslist.push_back(vlist[vid]);
	  }
	  reslist.push_back(vlist.back());
	  vlist.swap(reslist);

	  //quadratic simplification
	  
	  //find the points with smallest qem
	  while(vlist.size() > target){
	       int medge;
	       double minqem = 1e100;
	       Vector2d newpoint;
	       for(int eid=0; eid<vlist.size()-1; ++eid){
		    const int ind1 = (eid - 1 + vlist.size()) % (int)vlist.size();
		    const int ind2 = (eid + 2) % (int)vlist.size();
		    Vector2d line1 = vlist[eid] - vlist[ind1];
		    Vector2d line2 = vlist[eid+1] - vlist[eid];
		    Vector2d line3 = vlist[ind2] - vlist[eid+1];
		    Vector2d n1(-1*line1[1], line1[0]);
		    Vector2d n2(-1*line2[1], line2[0]);
		    Vector2d n3(-1*line3[1], line3[0]);
		    Matrix2d A;
		    Vector2d B;
		    const Vector2d& q1 = vlist[ind1];
		    const Vector2d& q2 = vlist[eid];
		    const Vector2d& q3 = vlist[eid+1];
		    A(0,0) = n1[0]*n1[0] + n2[0]*n2[0] + n3[0]*n3[0];
		    A(0,1) = n1[0]*n1[1] + n2[0]*n2[1] + n3[0]*n3[1];
		    A(1,0) = A(0,1);
		    A(1,1) = n1[1]*n1[1] + n2[1]*n2[1] + n3[1]*n3[1];
		    B[0] = n1[0]*(n1.dot(q1)) + n2[0]*(n2.dot(q2)) + n3[0]*(n3.dot(q3));
		    B[1] = n1[1]*(n1.dot(q1)) + n2[1]*(n2.dot(q2)) + n3[1]*(n3.dot(q3));
		    double c = n1.dot(q1) + n2.dot(q2) + n3.dot(q3);
		    c = c*c;
		    
		    Vector2d X = A.householderQr().solve(B);
		    double curqem = (X.transpose() * A).dot(X) - 2*X.dot(B) + c;
		    if(curqem < minqem){
			 medge = eid;
			 minqem = curqem;
			 newpoint = X;
		    }
	       }
	       vlist[medge] = newpoint;
	       vlist.erase(vlist.begin() + medge + 1);
	  }
     }
}  // namespace structured_indoor_modeling
  
