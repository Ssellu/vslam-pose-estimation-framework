#include "image_module/extract_features.hpp"

ImageProcessor::ImageProcessor(const std::string feature_type){
    draw = true;
    feature_type_ = feature_type;

    if (feature_type_.compare("ORB")==0){
        feature = cv::ORB::create(1000);
    }
    else if (feature_type_.compare("SIFT")==0){
        feature = cv::SIFT::create();
    }
    else if (feature_type_.compare("KAZE")==0){
        feature = cv::KAZE::create();
    }
    else if (feature_type_.compare("AKAZE")==0){
        feature = cv::AKAZE::create();
    }
    else if (feature_type_.compare("BRISK")==0){
        feature = cv::BRISK::create();
    }
    
}

ImageProcessor::~ImageProcessor(){
}

void ImageProcessor::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){  
    cv::Mat image_gray;
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptor_;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints_, descriptor_);

    descriptor = descriptor_.clone();
    if (draw){
        cv::Mat image_features;
        cv::drawKeypoints(image_bgr, keypoints_, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imwrite("./resources/images/result_" + feature_type_ + ".jpg", image_features);
    }
}