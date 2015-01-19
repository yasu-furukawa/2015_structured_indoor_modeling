#include "morphological_operation.h"
#include <cstdlib>
#include <iostream>

using namespace std;

namespace {

void GenerateKernel(const int kernel_width, vector<bool>* kernel) {
  const int kernel_width_half = kernel_width / 2;
  const int new_kernel_width = 2 * kernel_width_half + 1;
  kernel->clear();
  kernel->resize(new_kernel_width * new_kernel_width, false);

  int index = 0;
  for (int y = -kernel_width_half; y <= kernel_width_half; ++y) {
    for (int x = -kernel_width_half; x <= kernel_width_half; ++x, ++index) {
      if (abs(x) + abs(y) <= kernel_width_half)
        kernel->at(index) = true;
    }
  }
}

} // namespace

namespace image_process {

void Erode(const int width, const int height, const int kernel_width,
           vector<bool>* mask) {
  vector<bool> kernel;
  GenerateKernel(kernel_width, &kernel);
  
  Erode(width, height, kernel_width, kernel_width, kernel, mask);
}

void Erode(const int width, const int height, const int kernel_width, const int kernel_height,
           const vector<bool>& kernel, vector<bool>* mask) {
  if (width * height != mask->size()) {
    cerr << "Dimensions do not agree: " << width << ' ' << height << ' ' << width * height << ' '
         << mask->size() << endl;
    exit (1);
  }
  if (kernel_width * kernel_height != kernel.size()) {
    cerr << "Dimensions do not agree: " << kernel_width << ' ' << kernel_height << ' '
         << kernel_width * kernel_height << ' '
         << kernel.size() << endl;
    exit (1);
  }
  if (kernel_width % 2 != 1 || kernel_height % 2 != 1) {
    cerr << "Kernel dimensions must be odd: " << kernel_width << ' ' << kernel_height << endl;
    exit (1);
  }
  
  vector<bool> new_mask;
  new_mask.resize(width * height);

  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (x < kernel_width / 2  || width - kernel_width / 2 <= x ||
	  y < kernel_height / 2 || height - kernel_height / 2 <= y) {
	new_mask[index] = mask->at(index);
	continue;
      }
      
      // Check if a filter applies.
      bool keep = true;
      int kernel_index = 0;
      for (int j = -kernel_height / 2; j <= kernel_height / 2; ++j) {
	if (!keep)
	  break;
	const int ytmp = y + j;
	for (int i = -kernel_width / 2; i <= kernel_width / 2; ++i, ++kernel_index) {
	  const int xtmp = x + i;
	  if (kernel[kernel_index] && !mask->at(ytmp * width + xtmp)) {
	    keep = false;
	    break;
	  }
	}
      }
      if (!keep) {
	new_mask[index] = false;
      } else {
	new_mask[index] = true;
      }
    }
  }
  new_mask.swap(*mask);
}

void Dilate(const int width, const int height, const int kernel_width,
           vector<bool>* mask) {
  vector<bool> kernel;
  GenerateKernel(kernel_width, &kernel);
  
  Dilate(width, height, kernel_width, kernel_width, kernel, mask);
}

void Dilate(const int width, const int height, const int kernel_width, const int kernel_height,
           const vector<bool>& kernel, vector<bool>* mask) {
  if (width * height != mask->size()) {
    cerr << "Dimensions do not agree: " << width << ' ' << height << ' ' << width * height << ' '
         << mask->size() << endl;
    exit (1);
  }
  if (kernel_width * kernel_height != kernel.size()) {
    cerr << "Dimensions do not agree: " << kernel_width << ' ' << kernel_height << ' '
         << kernel_width * kernel_height << ' '
         << kernel.size() << endl;
    exit (1);
  }
  if (kernel_width % 2 != 1 || kernel_height % 2 != 1) {
    cerr << "Kernel dimensions must be odd: " << kernel_width << ' ' << kernel_height << endl;
    exit (1);
  }
  
  vector<bool> new_mask;
  new_mask.resize(width * height);

  int index = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x, ++index) {
      if (x < kernel_width / 2  || width - kernel_width / 2 <= x ||
	  y < kernel_height / 2 || height - kernel_height / 2 <= y) {
	new_mask[index] = mask->at(index);
	continue;
      }
      
      // Check if a filter applies.
      bool keep = false;
      int kernel_index = 0;
      for (int j = -kernel_height / 2; j <= kernel_height / 2; ++j) {
	if (keep)
	  break;
	const int ytmp = y + j;
	for (int i = -kernel_width / 2; i <= kernel_width / 2; ++i, ++kernel_index) {
	  const int xtmp = x + i;
	  if (kernel[kernel_index] && mask->at(ytmp * width + xtmp)) {
	    keep = true;
	    break;
	  }
	}
      }
      if (keep) {
	new_mask[index] = true;
      } else {
	new_mask[index] = false;
      }
    }
  }
  new_mask.swap(*mask);
}

void Open(const int width, const int height, const int kernel_width,
	  std::vector<bool>* mask) {
  vector<bool> kernel;
  GenerateKernel(kernel_width, &kernel);

  Open(width, height, kernel_width, kernel_width, kernel, mask);
}

void Open(const int width, const int height, const int kernel_width, const int kernel_height,
	  const std::vector<bool>& kernel, std::vector<bool>* mask) {
  Erode(width, height, kernel_width, kernel_height, kernel, mask);
  Dilate(width, height, kernel_width, kernel_height, kernel, mask);
}

void Close(const int width, const int height, const int kernel_width,
           std::vector<bool>* mask) {
  vector<bool> kernel;
  GenerateKernel(kernel_width, &kernel);

  Close(width, height, kernel_width, kernel_width, kernel, mask);
}

void Close(const int width, const int height, const int kernel_width, const int kernel_height,
           const std::vector<bool>& kernel, std::vector<bool>* mask) {
  Dilate(width, height, kernel_width, kernel_height, kernel, mask);
  Erode(width, height, kernel_width, kernel_height, kernel, mask);
}

}  // namespace image_process
