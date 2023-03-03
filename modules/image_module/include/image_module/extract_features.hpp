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

class Extractor{
public:
    Extractor();
    ~Extractor();
    virtual void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    virtual void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
protected:
    cv::Ptr<cv::Feature2D> feature;
    bool draw;
};

class ORB : public Extractor{
public:
    ORB();
    ORB(const int nFeatures);
    ~ORB();
    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
private:
};

class AKAZE : public Extractor{
public:
    AKAZE();
    AKAZE(const int nFeatures);
    ~AKAZE();
    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
private:
};

class KAZE : public Extractor{
public:
    KAZE();
    KAZE(const int nFeatures);
    ~KAZE();
    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
private:
};

class SIFT : public Extractor{
public:
    SIFT();
    SIFT(const int nFeatures);
    ~SIFT();
    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
private:
};

class BRISK : public Extractor{
public:
    BRISK();
    BRISK(const int nFeatures);
    ~BRISK();
    void detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor);
    void drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints);
private:
};

#endif //EXTRACT_FEATURES_HPP