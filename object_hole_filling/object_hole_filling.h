#pragma once

#include <Eigen/Eigen>
#include "../base/point_cloud.h"
#include "../base/panorama.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "MRF/mrf.h"
#include "MRF/GCoptimization.h"


//convert a Mat image to SLIC image
void MatToImagebuffer(const cv::Mat image, std::vector <unsigned int> &imagebuffer);

//convert a SLIC image to a Mat
void ImagebufferToMat(const std::vector<unsigned int>&imagebuffer,const int imgwidth, const int imgheight, cv::Mat &image);

//convert pixel label to label group

void labelTolabelgroup(const std::vector<int>& labels, std::vector< std::vector<int> >&labelgroup, int numgroup);

bool visibilityTest(const structured_indoor_modeling::Point &pt, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, int depthwidth);

int groupObject(const structured_indoor_modeling::PointCloud &point_cloud, std::vector <std::vector <int> >& objectgroup);

void getSuperpixelLabel(const structured_indoor_modeling::PointCloud &point_cloud, const std::vector<int>&objectgroup, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, const std::vector<int>& superpixel,const std::vector< std::vector<int> >&labelgroup, std::vector<int> &superpixelConfidence, int superpixelnum);

void pairSuperpixel(const std::vector <int> &labels, int width, int height, std::map<std::pair<int,int>, int> &pairmap);

void MRFOptimizeLabels(const std::vector<int>&superpixelConfidence,  const std::map<std::pair<int,int>,int> &pairmap, float smoothweight, std::vector<int>&superpixelLabel);










