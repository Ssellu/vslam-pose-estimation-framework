#ifndef EXTRACT_FEATURES_HPP
#define EXTRACT_FEATURES_HPP

#include "opencv2/opencv.hpp"
#include <algorithm>
#include <time.h>
#include <string>

class ImageProcessor{
public:
    ImageProcessor(const std::string feature_type);
    ~ImageProcessor();
    // void detectAndCompute(const cv::Mat& image_bgr);

    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);

private:
    std::string feature_type_;
    bool draw;
    cv::Ptr<cv::Feature2D> feature;
};


#endif //EXTRACT_FEATURES_HPP