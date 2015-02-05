#include "depth_filling.h"
#include "../base/panorama.h"
#include "../base/point_cloud.h"
#include "../base/file_io.h"

using namespace std;
using namespace Eigen;
using namespace cv;

namespace structured_indoor_modeling{

     vector<double> JacobiMethod(const vector< vector<double> >& A, const vector<double>& B, int max_iter = 300){
    vector<double>solution(B.size());
    for(auto& v:solution) v=0.0;
    for(int iter=0;iter<max_iter;iter++){
      for(int i=0;i<solution.size();i++){
	double sum = 0.0;
	for(int j=0;j<solution.size();j++){
	  if(j!=i)
	    sum += A[i][j] * solution[j];
	}
	solution[i] = (B[i] - sum) / A[i][i];
      }
      //evaluate the result
      double ssd = 0;
      for(int i=0;i<solution.size();i++){
	double tempsum = 0.0;
	for(int j=0;j<solution.size();j++){
	  tempsum += A[i][j] * solution[j];
	}
	tempsum -= B[i];
	ssd += tempsum * tempsum;
      }
      ssd = sqrt(ssd);
#if 1
      cout<<"Iteration "<<iter<<", ssd: "<<ssd<<endl;
#endif
      if(ssd < 0.001)
	break;
    }
    return solution;
  }
  
    void DepthFilling::Init(const PointCloud& point_cloud, const Panorama &panorama){
	depthwidth = panorama.DepthWidth();
	depthheight = panorama.DepthHeight();
	depthmap.clear();
	depthmap.resize(depthwidth * depthheight);
	for(auto &v :depthmap)
	    v = -1.0;
	//project the point cloud to the panorama and get the depth
	max_depth = -1e100;
	min_depth = 1e100;
	for(int point=0;point<point_cloud.GetNumPoints();++point){
	    Vector3d globalposition = point_cloud.GetPoint(point).position;
	    Vector2d pixel = panorama.Project(globalposition);
	    Vector2d depth_pixel = panorama.RGBToDepth(pixel);
	    Vector3d localp = panorama.GlobalToLocal(globalposition);
	    int depthx = round(depth_pixel[0]+0.5);
	    int depthy = round(depth_pixel[1]+0.5);
	    if(depthx<0 || depthx>= depthwidth || depthy<0 || depthy>=depthheight)
		continue;
	    double depth = sqrt(localp[0]*localp[0] + localp[1]*localp[1] + localp[2]*localp[2]);
	    depthmap[depthy*depthwidth + depthx] = depth;

	     if(depth < min_depth)
		 min_depth = depth;
	     if(depth > max_depth)
		 max_depth = depth;
	 }
     }

    inline bool DepthFilling::insideDepth(int x,int y){
	return x>=0 && x<depthwidth && y>=0 && y<depthheight;
    }
    
     void DepthFilling::fill_hole(const Panorama& panorama){
	 int invalidnum= 0;

	 //invalidcoord: size of invalidnum
	 //invalidindx: size of depthnum
	 vector <int> invalidcoord;
	 vector <int> invalidindex(depthmap.size());
	 
	 for(int i=0;i<depthmap.size();i++){
	     Vector2d depth_pixel((double)(i%depthwidth), (double)(i/depthwidth));
	     Vector2d color_pixel = panorama.DepthToRGB(depth_pixel);
	     if(panorama.GetRGB(color_pixel) == Vector3f(0,0,0))
		 continue;
	     if(depthmap[i] < 0){
		 invalidnum ++;
		 invalidcoord.push_back(i);
		 invalidindex[i] = invalidnum - 1;
	     }
	 }
	 cout<<"Invalid depth num:"<<invalidnum<<endl;
	 //construct matrix A and B
	 vector< vector<double> > A(invalidnum);
	 vector <double> B(invalidnum);
	 for(int i=0;i<invalidnum;i++){
	     A[i].resize(invalidnum);
	     for(int j=0;j<invalidnum;j++){
		 A[i][j] = 0.0;
	     }
	     B[i] = 0.0;
	 }
	 
	 for(int i=0;i<invalidnum;i++){
	     //(x,y) is the coordinate of invalid pixel
	     int x = invalidcoord[i] % depthwidth;
	     int y = invalidcoord[i] / depthwidth;
	     int count = 0;
	     if(insideDepth(x-1,y)){
		 count++;
		 if(depthmap[y*depthwidth + x-1] <0 )
		     A[i][invalidindex[y*depthwidth+x-1]] = -1;
		 else
		     B[i] += depthmap[y*depthwidth+x-1];
	     }
	     if(insideDepth(x+1,y)){
		 count++;
		 if(depthmap[y*depthwidth + x+1] <0 )
		     A[i][invalidindex[y*depthwidth+x+1]] = -1;
		 else
		     B[i] += depthmap[y*depthwidth+x+1];
	     }
	     if(insideDepth(x,y-1)){
		 count++;
		 if(depthmap[(y-1)*depthwidth + x] <0 )
		     A[i][invalidindex[(y-1)*depthwidth+x]] = -1;
		 else
		     B[i] += depthmap[(y-1)*depthwidth+x];
	     }
	     if(insideDepth(x,y+1)){
		 count++;
		 if(depthmap[(y+1)*depthwidth + x] <0 )
		     A[i][invalidindex[(y+1)*depthwidth+x]] = -1;
		 else
		     B[i] += depthmap[(y+1)*depthwidth+x];
	     }
	     A[i][i] = (double)count;
	 }

	 //solve the linear problem with Jacobi iterative method
	 vector <double> solution = JacobiMethod(A,B);

	 //copy the result to original depthmap
	 for(int i=0;i<invalidnum;i++){
	     depthmap[invalidcoord[i]] = solution[i];
	 }
     }

    void DepthFilling::SaveDepthmap(string path){
	Mat depthimage(depthheight,depthwidth, CV_8UC3);
	for(int i=0;i<depthmap.size();i++){
	    if(depthmap[i] < 0){
		depthimage.at<Vec3b>(i/depthwidth, i%depthwidth) = Vec3b(0,0,255);
		continue;
	    }
	    int curv = (int)(depthmap[i] - min_depth) / (max_depth - min_depth) * 255;
	    Vec3b curpix((uchar)curv, (uchar)curv, (uchar)curv);
	    depthimage.at<Vec3b>(i / depthwidth, i%depthwidth) = curpix;
	}
  
	imwrite(path, depthimage);
	waitKey(10);
    }

 
}//namespace
