#pragma once

#include <Eigen/Eigen>
#include "../base/point_cloud.h"
#include "../base/panorama.h"
#include <iostream>
#include <opencv2/opencv.hpp>

//convert a Mat image to SLIC image
void MatToImagebuffer(const cv::Mat image, std::vector <unsigned int> &imagebuffer);

//convert a SLIC image to a Mat
void ImagebufferToMat(const std::vector<unsigned int>&imagebuffer,const int imgwidth, const int imgheight, cv::Mat &image);

//convert pixel label to label group

void labelTolabelgroup(const std::vector<int>& labels, std::vector< std::vector<int> >&labelgroup, int numgroup);

bool visibilityTest(const structured_indoor_modeling::Point &pt, const structured_indoor_modeling::Panorama &panorama, const std::vector<double> &depthmap, int depthwidth);



















