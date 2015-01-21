#ifndef IMAGE_PROCESS_MORPHOLOGICAL_OPERATION_H_
#define IMAGE_PROCESS_MORPHOLOGICAL_OPERATION_H_

#include <vector>

namespace image_process {

void Erode(const int width, const int height, const int kernel_width,
           std::vector<bool>* mask);

void Erode(const int width, const int height, const int kernel_width, const int kernel_height,
           const std::vector<bool>& kernel, std::vector<bool>* mask);

void Dilate(const int width, const int height, const int kernel_width,
	    std::vector<bool>* mask);

void Dilate(const int width, const int height, const int kernel_width, const int kernel_height,
           const std::vector<bool>& kernel, std::vector<bool>* mask);

void Open(const int width, const int height, const int kernel_width,
	  std::vector<bool>* mask);

void Open(const int width, const int height, const int kernel_width, const int kernel_height,
	  const std::vector<bool>& kernel, std::vector<bool>* mask);

void Close(const int width, const int height, const int kernel_width,
           std::vector<bool>* mask);

void Close(const int width, const int height, const int kernel_width, const int kernel_height,
           const std::vector<bool>& kernel, std::vector<bool>* mask);


}  // namespace image_process

#endif  // IMAGE_PROCESS_MORPHOLOGICAL_OPERATION_H_
