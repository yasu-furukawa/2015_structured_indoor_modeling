#pragma once

#include <Eigen/Eigen>
#include "../base/point_cloud.h"
#include "../base/panorama.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "MRF/mrf.h"
#include "MRF/GCoptimization.h"
#include "depth_filling.h"


//convert a Mat image to SLIC image
void MatToImagebuffer(const cv::Mat image, std::vector <unsigned int> &imagebuffer);

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

void getSuperpixelConfidence(const structured_indoor_modeling::PointCloud &point_cloud, const std::vector<int>&objectgroup, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, const std::vector<int>& superpixel,const std::vector< std::vector<int> >&labelgroup, std::vector<double> &superpixelConfidence, int superpixelnum);


void pairSuperpixel(const std::vector <int> &labels, int width, int height, std::map<std::pair<int,int>, int> &pairmap);

void MRFOptimizeLabels(const std::vector<int>&superpixelConfidence,  const std::map<std::pair<int,int>,int> &pairmap,const std::vector< Eigen::Vector3d >&averageRGB, float smoothweight, std::vector<int>&superpixelLabel);

void MRFOptimizeLabels_multiLayer(const std::vector< std::vector<double> >&superpixelConfidence, const std::map<std::pair<int,int>,int> &pairmap, const std::vector< Eigen::Vector3d> &averageRGB,  float smoothweight, int numlabels, std::vector <int> &superpixelLabel, structured_indoor_modeling::DepthFilling &objectdepth);

void BackProjectObject(const structured_indoor_modeling::Panorama &panorama, const structured_indoor_modeling::DepthFilling &depth, const structured_indoor_modeling::PointCloud &objectcloud, const std::vector< std::vector<int> >&objectgroup,  const std::vector<int>&segmentation, const std::vector< std::vector<int> >&labelgroup, structured_indoor_modeling::PointCloud &resultcloud, int roomid);

void mergeVertices(structured_indoor_modeling::PointCloud &pc, int resolution);



















