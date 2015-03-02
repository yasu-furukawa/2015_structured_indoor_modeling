#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>


namespace structured_indoor_modeling{

  class FileIO;
  class PointCloud;
  class Panorama;

  std::vector <double> JacobiMethod(const std::vector <std::vector<double> >& A, const std::vector<double>& B, int max_iter);


  class DepthFilling{
  public:
    DepthFilling(){}
    void Init(const PointCloud &point_cloud, const Panorama &panorama);
    void fill_hole(const Panorama& panorama);
    inline bool insideDepth(int x,int y);
    //just for debugging
    void SaveDepthmap(std::string path);
    void SaveDepthFile(std::string path);
    double GetDepth(double x,double y) const;
    double GetDepth(int x,int y) const;
    bool ReadDepthFromFile(std::string path);
    const std::vector<double>& GetDepthmap() const{return depthmap;}
  private:
    std::vector <double> depthmap;
    int depthwidth;
    int depthheight;
    double max_depth;
    double min_depth;
  };

} // namespace











