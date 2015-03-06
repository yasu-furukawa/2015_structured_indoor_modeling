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
    void setMask(int id, bool maskv);
    void setMask(int x, int y, bool maskv);
    void setMask(std::vector <bool> maskv);
    void fill_hole(const Panorama& panorama);

    inline bool insideDepth(int x,int y){
      return x>=0 && x<depthwidth && y>=0 && y<depthheight && mask[y*depthwidth + x];
    }
    
    void SaveDepthmap(std::string path);
    void SaveDepthFile(std::string path);
    double GetDepth(double x,double y) const;
    double GetDepth(int x,int y) const;
    bool ReadDepthFromFile(std::string path);
    const std::vector<double>& GetDepthmap() const{return depthmap;}
  private:
    std::vector <double> depthmap;
    std::vector <bool> mask;
    int depthwidth;
    int depthheight;
    double max_depth;
    double min_depth;
  };

} // namespace











