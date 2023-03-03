#include "image_module/extract_features.hpp"

int main(int argc, char **argv)
{
    cv::Mat image = cv::imread("./resources/images/stitch_image1_1.jpg");

    cv::Mat descriptor;
    std::string feature_type = "ORB";
    Extractor* extractor;
    if(argc>1){
        feature_type = argv[1];
    }

    if (feature_type.compare("ORB")==0)
        extractor = new ORB(1000);

    else if (feature_type.compare("AKAZE")==0)
        extractor = new AKAZE();

    else if (feature_type.compare("KAZE")==0)
        extractor = new KAZE();

    else if (feature_type.compare("SIFT")==0)
        extractor = new SIFT();

    else if (feature_type.compare("BRISK")==0)
        extractor = new BRISK();
    
    extractor->detectAndCompute(image, descriptor);

    std::cout<< descriptor.size() << std::endl;
    return 0;
}