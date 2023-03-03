#include "image_module/extract_features.hpp"

int main(int argc, char **argv)
{
    cv::Mat image = cv::imread("./resources/images/stitch_image1_1.jpg");

    cv::Mat descriptor;
    std::string feature_type = "ORB";

    if(argc>1){
        feature_type = argv[1];
    }

    ImageProcessor imageProcessor(feature_type);
    imageProcessor.detectAndCompute(image, descriptor);

    std::cout<< descriptor.size() << std::endl;
    return 0;
}