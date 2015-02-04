#include "object_hole_filling.h"

using namespace std;
using namespace cv;


void MatToImagebuffer(const Mat image, vector<unsigned int>&imagebuffer){
  if(!image.data){
    cout << "invlid image"<<endl;
    exit(-1);
  }
  int imgheight = image.rows;
  int imgwidth = image.cols;
  imagebuffer.clear();
  imagebuffer.resize(imgheight * imgwidth);
  for(int y=0;y<imgheight;y++){
    for(int x=0;x<imgwidth;x++){
      Vec3b curpix = image.at<Vec3b>(y,x);
      int ind = y*imgwidth + x;
      imagebuffer[ind] = (unsigned int)255*256*256*256 + (unsigned int)curpix[0]*256*256 + (unsigned int)curpix[1]*256 + (unsigned int)curpix[2];
    }
  }
}


void ImagebufferToMat(const vector <unsigned int>imagebuffer,const int imgwidth,const int imgheight,  Mat& image){
  if(imagebuffer.size() != imgwidth * imgheight){
    cout << "Sizes don't agree!"<<endl;
    exit(-1);
  }
  image.release();
  image = Mat(imgheight,imgwidth,CV_8UC3);
  for(int y=0;y<imgheight;y++){
    for(int x=0;x<imgwidth;x++){
      Vec3b curpix;
      curpix[0] = imagebuffer[y*imgwidth+x] >> 16 & 0xff;
      curpix[1] = imagebuffer[y*imgwidth+x] >> 8 & 0xff;
      curpix[2] = imagebuffer[y*imgwidth+x] & 0xff;
      image.at<Vec3b>(y,x) = curpix;
    }
  }
}



