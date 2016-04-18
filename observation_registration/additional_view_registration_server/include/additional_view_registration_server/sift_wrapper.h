#ifndef __SIFT_WRAPPER__
#define __SIFT_WRAPPER__

#include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <siftgpu/SiftGPU.h>
#include <GL/gl.h>
#include <string>
#include <Eigen/Eigen>

class SIFTWrapper{
public:
    SIFTWrapper();
    ~SIFTWrapper();

    void extractSIFT(const cv::Mat& image, int& num_desc, std::vector<float>& desc, std::vector<SiftGPU::SiftKeypoint>& keypoints);
    void matchSIFT(const int& num_desc1, const int& num_desc2,
                   const std::vector<float>& desc1, const std::vector<float>& desc2,
                   const std::vector<SiftGPU::SiftKeypoint>& keypoints1, const std::vector<SiftGPU::SiftKeypoint>& keypoints2,
                   std::vector<std::pair<SiftGPU::SiftKeypoint, SiftGPU::SiftKeypoint>>& matches);

private:
    bool initialized;
    SiftGPU  *sift;
    SiftMatchGPU *matcher;
};


#endif
