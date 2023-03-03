#include "image_module/extract_features.hpp"

Extractor::Extractor(){
}

Extractor::~Extractor(){
}

void Extractor::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    std::cout << "detect feature from image, and compute descriptor" << std::endl;
}

void Extractor::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint> &keypoints){
    std::cout << "drawKeypoints on image, and save result" << std::endl;
}

ORB::ORB() : Extractor(){
    feature = cv::ORB::create(500);
    draw = false;
}

ORB::ORB(const int nFeatures=500) : Extractor(){
    feature = cv::ORB::create(nFeatures);
    draw = false;
}

ORB::~ORB(){
}

void ORB::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    cv::Mat image_gray;
    
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptor);
    if (draw){
        drawKeypoints(image_bgr, keypoints);
    }
}

void ORB::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints){
    cv::Mat image_features;
    cv::drawKeypoints(image_bgr, keypoints, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite("./resources/images/result_ORB.jpg", image_features);
}

AKAZE::AKAZE() : Extractor(){
    feature = cv::AKAZE::create();
    draw = false;
}

AKAZE::~AKAZE(){
}

void AKAZE::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    cv::Mat image_gray;
    
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptor);
    if (draw){
        drawKeypoints(image_bgr, keypoints);
    }
}

void AKAZE::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints){
    cv::Mat image_features;
    cv::drawKeypoints(image_bgr, keypoints, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite("./resources/images/result_AKAZE.jpg", image_features);
}

KAZE::KAZE() : Extractor(){
    feature = cv::KAZE::create();
    draw = false;
}

KAZE::~KAZE(){
}

void KAZE::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    cv::Mat image_gray;
    
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptor);
    if (draw){
        drawKeypoints(image_bgr, keypoints);
    }
}

void KAZE::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints){
    cv::Mat image_features;
    cv::drawKeypoints(image_bgr, keypoints, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite("./resources/images/KAZE.jpg", image_features);
}

SIFT::SIFT() : Extractor(){
    feature = cv::SIFT::create();
    draw = false;
}

SIFT::~SIFT(){
}

void SIFT::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    cv::Mat image_gray;
    
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptor);
    if (draw){
        drawKeypoints(image_bgr, keypoints);
    }
}

void SIFT::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints){
    cv::Mat image_features;
    cv::drawKeypoints(image_bgr, keypoints, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite("./resources/images/SIFT.jpg", image_features);
}

BRISK::BRISK() : Extractor(){
    feature = cv::BRISK::create();
    draw = false;
}

BRISK::~BRISK(){
}

void BRISK::detectAndCompute(const cv::Mat& image_bgr, cv::Mat& descriptor){
    cv::Mat image_gray;
    
    cv::cvtColor(image_bgr, image_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;

    feature->detectAndCompute(image_gray, cv::Mat(), keypoints, descriptor);
    if (draw){
        drawKeypoints(image_bgr, keypoints);
    }
}

void BRISK::drawKeypoints(const cv::Mat& image_bgr, const std::vector<cv::KeyPoint>& keypoints){
    cv::Mat image_features;
    cv::drawKeypoints(image_bgr, keypoints, image_features, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imwrite("./resources/images/BRISK.jpg", image_features);
}