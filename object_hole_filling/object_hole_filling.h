#pragma once

#include <Eigen/Eigen>
#include "../base/point_cloud.h"
#include "../base/file_io.h"
#include "../base/panorama.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "MRF/mrf.h"
#include "MRF/GCoptimization.h"
#include "depth_filling.h"


void initPanorama(const structured_indoor_modeling::FileIO &file_io, std::vector<structured_indoor_modeling::Panorama>&panorama, std::vector<std::vector<int> >&labels, const int expected_num, std::vector<int>&numlabels, std::vector<structured_indoor_modeling::DepthFilling>&depth, int &imgwidth, int &imgheight, const int startid, const int endid);

//convert a Mat image to SLIC image
void MatToImagebuffer(const cv::Mat image, std::vector <unsigned int> &imagebuffer);

//for debuging
void saveConfidence(const std::vector< std::vector<double> >&superpixelConfidence, const std::vector<int>&labels, const int imgwidth, const int imgheight, const int id, const int roomid);
void saveOptimizeResult(const structured_indoor_modeling::Panorama &panorama, const std::vector<int> &superpixelLabel, const std::vector<int>&labels, const int id, const int roomid);

//convert a SLIC image to a Mat
void ImagebufferToMat(const std::vector<unsigned int>&imagebuffer,const int imgwidth, const int imgheight, cv::Mat &image);

void RGB2HSV(double r,double g, double b, double &h, double &s, double &v);

void HSV2RGB(double h, double s, double v, double r, double &g, double &b);
//this funtion will read the object cloud and floor_wall cloud. It'll treat floor_wall as another object
void ReadObjectCloud(const structured_indoor_modeling::FileIO& file_io, std::vector< structured_indoor_modeling::PointCloud>&objectCloud, std::vector< std::vector< std::vector<int> > >&objectgroup, std::vector< std::vector<double> >&objectVolume);

//convert pixel label to label group
void labelTolabelgroup(const std::vector<int>& labels,const structured_indoor_modeling::Panorama &panorama, std::vector< std::vector<int> >&labelgroup,std::vector< Eigen::Vector3d >&averageRGB, int numgroup);

inline double gaussianFunc(double x, double sigma){
    return std::exp(-1*(x*x/(2*sigma*sigma)));
}

inline double sigmaFunc(double x, double offset, double maxv, double scale){
    return maxv/(1+std::exp(scale * (x - offset)));
}

bool visibilityTest(const structured_indoor_modeling::Point &pt, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, int depthwidth);

int groupObject(const structured_indoor_modeling::PointCloud &point_cloud, std::vector <std::vector <int> >& objectgroup, std::vector<double>&objectVolume);    //objectVolume: volume of the boundingbox of each object

void getSuperpixelConfidence(const structured_indoor_modeling::PointCloud &point_cloud, const std::vector<int>&objectgroup, const structured_indoor_modeling::Panorama &panorama, const structured_indoor_modeling::DepthFilling& depthmap, const std::vector<int>& superpixel,const std::vector< std::vector<int> >&labelgroup,const std::map<std::pair<int,int>,int>& pairmap, std::vector<double> &superpixelConfidence, const int superpixelnum, const int erodeiter);


void pairSuperpixel(const std::vector <int> &labels, int width, int height, std::map<std::pair<int,int>, int> &pairmap);

void MRFOptimizeLabels(const std::vector<int>&superpixelConfidence,  const std::map<std::pair<int,int>,int> &pairmap,const std::vector< Eigen::Vector3d >&averageRGB, float smoothweight, std::vector<int>&superpixelLabel);

void MRFOptimizeLabels_multiLayer(const std::vector< std::vector<double> >&superpixelConfidence, const std::map<std::pair<int,int>,int> &pairmap, const std::vector< Eigen::Vector3d> &averageRGB,  float smoothweight, int numlabels, std::vector <int> &superpixelLabel);

void backProjectObject(const structured_indoor_modeling::Panorama &panorama, const structured_indoor_modeling::DepthFilling &depth, const structured_indoor_modeling::PointCloud &objectcloud, const std::vector< std::vector<int> >&objectgroup,  const std::vector<int>&segmentation, const std::vector< std::vector<int> >&labelgroup, structured_indoor_modeling::PointCloud &resultcloud);

//void BackProjectObject(const std::vector<structured_indoor_modeling::Panorama>& panorama, const std::vector

void mergeObject(const structured_indoor_modeling::Panorama &panorama, const structured_indoor_modeling::DepthFilling &depth, const structured_indoor_modeling::PointCloud &objectcloud, const std::vector< std::vector<int> >&objectgroup,  const std::vector<int>&segmentation, const std::vector< std::vector<int> >&labelgroup, std::vector <std::list< structured_indoor_modeling::PointCloud> > &objectlist);


void cleanObjects(structured_indoor_modeling::PointCloud &pc, const double min_volume);

void ICP(structured_indoor_modeling::PointCloud &src, const structured_indoor_modeling::PointCloud &tgt, const int num_iter = 10);

void mergeVertices(structured_indoor_modeling::PointCloud &pc, int resolution);



















