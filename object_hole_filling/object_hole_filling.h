#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//convert a Mat image to SLIC image
void MatToImagebuffer(const Mat image, vector <unsigned int> &imagebuffer);

//convert a SLIC image to a Mat
void ImagebufferToMat(const vector<unsigned int>imagebuffer,const int imgwidth, const int imgheight, Mat &image);

//read point cloud and convert to depth map
