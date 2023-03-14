#include "base_framepoint_generator.h"

namespace proslam {

Detector::Detector(){}
Detector::~Detector(){}

//FAST
FastDetector::FastDetector() : Detector(){
  extractor = cv::FastFeatureDetector::create();
}
FastDetector::FastDetector(const double _threshold) : Detector(){
  extractor = cv::FastFeatureDetector::create(std::rint(_threshold));
}
FastDetector::~FastDetector(){}
double FastDetector::getThreshold(){
  return extractor->getThreshold();
}
void FastDetector::setThreshold(const double _threshold){
  // std::rint
  extractor->setThreshold(std::rint(_threshold));
}
void FastDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
}

//AKAZE
AkazeDetector::AkazeDetector() : Detector(){
  extractor = cv::AKAZE::create();
}
AkazeDetector::AkazeDetector(const double _threshold) : Detector(){
  // int descriptor_type=AKAZE::DESCRIPTOR_MLDB, int descriptor_size=0, int descriptor_channels=3,
  // float threshold=0.001f, int nOctaves=4, int nOctaveLayers=4, int diffusivity=KAZE::DIFF_PM_G2
  extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, _threshold/div_value);
  // extractor = cv::AKAZE::create();
}
AkazeDetector::~AkazeDetector(){}
double AkazeDetector::getThreshold(){
  return extractor->getThreshold()*div_value;
  // return 20;
}
void AkazeDetector::setThreshold(const double _threshold){
  extractor->setThreshold(_threshold/div_value);
  // return;
}
void AkazeDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
  std::cout<<"keypoint size:" <<_keypoints.size()<<std::endl;
}

//ORB
OrbDetector::OrbDetector() : Detector(){
  extractor = cv::ORB::create(5000);
}
OrbDetector::OrbDetector(const double _threshold) : Detector(){
  // nfeatures, scaleFactor, nlevels, edgethreshold, firstlevel, wta_k, scoretype, patchsize, fastthreshold
  extractor = cv::ORB::create(5000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, _threshold);
}
OrbDetector::~OrbDetector(){}
double OrbDetector::getThreshold(){
  return extractor->getFastThreshold();
  // return extractor->getEdgeThreshold();
}
void OrbDetector::setThreshold(const double _threshold){
  extractor->setFastThreshold(std::rint(_threshold));
  // extractor->setEdgeThreshold(_threshold);
}
void OrbDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
}

//KAZE
KazeDetector::KazeDetector() : Detector(){
  extractor = cv::KAZE::create();
}
KazeDetector::KazeDetector(const double _threshold) : Detector(){
  // bool extended=false, bool upright=false, float threshold=0.001f, int nOctaves=4,
  // int nOctaveLayers=4, int diffusivity=KAZE::DIFF_PM_G
  extractor = cv::KAZE::create(0, 0, _threshold/div_value);
}
KazeDetector::~KazeDetector(){}
double KazeDetector::getThreshold(){
  return extractor->getThreshold()*div_value;
}
void KazeDetector::setThreshold(const double _threshold){
  extractor->setThreshold(_threshold/div_value);
}
void KazeDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
  std::cout<<"keypoint size:" <<_keypoints.size()<<std::endl;
}

//SIFT
SiftDetector::SiftDetector() : Detector(){
  extractor = cv::xfeatures2d::SIFT::create();
}
SiftDetector::SiftDetector(const double _threshold) : Detector(){
  // int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6)
  threshold = _threshold / div_value;
  extractor = cv::xfeatures2d::SIFT::create(0,3, 0.04, threshold);
}
SiftDetector::~SiftDetector(){}
double SiftDetector::getThreshold(){
  return threshold * div_value;
}
void SiftDetector::setThreshold(const double _threshold){
  threshold = _threshold / div_value;
  extractor = cv::xfeatures2d::SIFT::create(0,3, 0.04, threshold);
}
void SiftDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
  std::cout<<"keypoint size:" <<_keypoints.size()<<std::endl;
}

//BRISK
BriskDetector::BriskDetector() : Detector(){
  extractor = cv::BRISK::create();
  threshold = 30;
}
BriskDetector::BriskDetector(const double _threshold) : Detector(){
  // int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6)
  threshold = _threshold*1.5;
  extractor = cv::BRISK::create(threshold);
}
BriskDetector::~BriskDetector(){}
double BriskDetector::getThreshold(){
  return threshold/1.5;
}
void BriskDetector::setThreshold(const double _threshold){
  threshold = _threshold*1.5;
  extractor = cv::BRISK::create(threshold);
}
void BriskDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
  std::cout<<"keypoint size:" <<_keypoints.size() << "th:" << threshold <<std::endl;
}

AgastDetector::AgastDetector() : Detector(){
  extractor = cv::AgastFeatureDetector::create();
  threshold = 30;
}
AgastDetector::AgastDetector(const double _threshold) : Detector(){
  // int nfeatures=0, int nOctaveLayers=3, double contrastThreshold=0.04, double edgeThreshold=10, double sigma=1.6)
  threshold = _threshold*1.5;
  extractor = cv::AgastFeatureDetector::create(threshold);
}
AgastDetector::~AgastDetector(){}
double AgastDetector::getThreshold(){
  return extractor->getThreshold();
  // return threshold/1.5;
}
void AgastDetector::setThreshold(const double _threshold){
  // threshold = _threshold*1.5;
  extractor->setThreshold(_threshold);
}
void AgastDetector::detect(const cv::Mat& _image, std::vector<cv::KeyPoint>& _keypoints){
  extractor->detect(_image, _keypoints);
  std::cout<<"keypoint size:" <<_keypoints.size() << "th:" << threshold <<std::endl;
}

BaseFramePointGenerator::BaseFramePointGenerator(BaseFramePointGeneratorParameters* parameters_): _parameters(parameters_) {
  LOG_INFO(std::cerr << "BaseFramePointGenerator::BaseFramePointGenerator|constructed" << std::endl)
}

void  BaseFramePointGenerator::configure(){
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_left);

  _number_of_rows_image            = _camera_left->numberOfImageRows();
  _number_of_cols_image            = _camera_left->numberOfImageCols();
  _focal_length_pixels             = _camera_left->cameraMatrix()(0,0);
  _principal_point_offset_u_pixels = _camera_left->cameraMatrix()(0,2);
  _principal_point_offset_v_pixels = _camera_left->cameraMatrix()(1,2);
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|focal length (pixels): " << _focal_length_pixels << std::endl)

  //ds initialize feature matcher
  _feature_matcher_left.configure(_number_of_rows_image, _number_of_cols_image);

  //ds configure tracking window
  _projection_tracking_distance_pixels  = _parameters->maximum_projection_tracking_distance_pixels;
  _maximum_descriptor_distance_tracking = _parameters->maximum_descriptor_distance_tracking;

  //ds allocate descriptor extractor TODO enable further support and check BIT SIZES

  if (_parameters->descriptor_type == "BRIEF") {
    #ifdef SRRG_PROSLAM_HAS_OPENCV_CONTRIB
      _descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(DESCRIPTOR_SIZE_BYTES);
    #else
      LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: BRIEF"
                            << " is not available in current build, defaulting to ORB" << std::endl)
      _descriptor_extractor        = cv::ORB::create();
      _parameters->descriptor_type = "ORB";
    #endif

  }
  else if (_parameters->descriptor_type == "ORB") {
    _descriptor_extractor = cv::ORB::create();
  }
  else if (_parameters->descriptor_type == "BRISK") {
    _descriptor_extractor = cv::BRISK::create();
  }
  else if (_parameters->descriptor_type == "AKAZE"){
    _descriptor_extractor = cv::AKAZE::create();
  }
  else if (_parameters->descriptor_type == "KAZE"){
    _descriptor_extractor = cv::KAZE::create();
  }
  else if (_parameters->descriptor_type == "SIFT"){
    _descriptor_extractor = cv::xfeatures2d::SIFT::create();
  }
  else if (_parameters->descriptor_type == "FREAK") {
    #ifdef SRRG_PROSLAM_HAS_OPENCV_CONTRIB
        _descriptor_extractor = cv::xfeatures2d::FREAK::create();
    #else
        LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: FREAK"
                              << " is not available in current build, defaulting to ORB" << std::endl)
        _descriptor_extractor        = cv::ORB::create();
        _parameters->descriptor_type = "ORB";
    #endif
  }
  else {
    LOG_WARNING(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                          << " is not implemented, defaulting to ORB" << std::endl)
    _descriptor_extractor        = cv::ORB::create();
    _parameters->descriptor_type = "ORB";
  }

  //ds log chosen descriptor type and size
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|detector_type: " << _parameters->detector_type << std::endl)
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|descriptor_type: " << _parameters->descriptor_type
                     << " (memory: " << SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS << "b)" << std::endl)

  //ds allocate and initialize detector grid structure
  // _detectors           = new cv::Ptr<cv::Feature2D>*[_parameters->number_of_detectors_vertical];
  _detectors = new Detector**[_parameters->number_of_detectors_vertical];
  _detector_regions    = new cv::Rect*[_parameters->number_of_detectors_vertical];
  _detector_thresholds = new real*[_parameters->number_of_detectors_vertical];
  const real pixel_rows_per_detector = static_cast<real>(_number_of_rows_image)/_parameters->number_of_detectors_vertical;
  const real pixel_cols_per_detector = static_cast<real>(_number_of_cols_image)/_parameters->number_of_detectors_horizontal;
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    // _detectors[r]           = new cv::Ptr<cv::Feature2D>[_parameters->number_of_detectors_horizontal];
    _detectors[r] = new Detector*[_parameters->number_of_detectors_horizontal];
    _detector_regions[r]    = new cv::Rect[_parameters->number_of_detectors_horizontal];
    _detector_thresholds[r] = new real[_parameters->number_of_detectors_horizontal];

    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {
      if (_parameters->detector_type == "FAST")
        _detectors[r][c] = new FastDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "AKAZE")
        _detectors[r][c] = new AkazeDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "ORB")
        _detectors[r][c] = new OrbDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "KAZE")
        _detectors[r][c] = new KazeDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "SIFT")
        _detectors[r][c] = new SiftDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "BRISK")
        _detectors[r][c] = new BriskDetector(_parameters->detector_threshold_minimum);
      else if(_parameters->detector_type == "AGAST")
        _detectors[r][c] = new AgastDetector(_parameters->detector_threshold_minimum);
      else{
        _detectors[r][c] = new FastDetector(_parameters->detector_threshold_minimum);
        _parameters->descriptor_type = "ORB";
      }
      //ds consider small overlaps of detector image regions to avoid point discard at borders
      int32_t offset_width  = 0;
      int32_t offset_height = 0;
      if (_parameters->number_of_detectors_vertical > 1) {
        offset_height = 2;
      }
      if (_parameters->number_of_detectors_horizontal > 1) {
        offset_width = 2;
      }

      //ds determine detector region overlap (for consistent point detection)
      int32_t offset_r = 0;
      int32_t offset_c = 0;
      if (r > 0) {
        offset_r = -offset_height;

        //ds double offset for central regions (not at border)
        if (r < _parameters->number_of_detectors_vertical-1) {
          offset_height *= 2;
        }
      }
      if (c > 0) {
        offset_c = -offset_width;

        //ds double offset for central regions (not at border)
        if (c < _parameters->number_of_detectors_horizontal-1) {
          offset_width *= 2;
        }
      }

      //ds specify region
      _detector_regions[r][c] = cv::Rect(std::round(c*pixel_cols_per_detector)+offset_c,
                                         std::round(r*pixel_rows_per_detector)+offset_r,
                                         pixel_cols_per_detector+offset_width,
                                         pixel_rows_per_detector+offset_height);
      _detector_thresholds[r][c] = 0;
    }
  }
  _number_of_detectors = _parameters->number_of_detectors_vertical*_parameters->number_of_detectors_horizontal;
  _mean_detector_threshold = _parameters->detector_threshold_minimum;

  //ds compute binning configuration
  _number_of_cols_bin = std::floor(static_cast<real>(_camera_left->numberOfImageCols())/_parameters->bin_size_pixels)+1;
  _number_of_rows_bin = std::floor(static_cast<real>(_camera_left->numberOfImageRows())/_parameters->bin_size_pixels)+1;

  //ds compute target number of points
  _target_number_of_keypoints = _number_of_cols_bin*_number_of_rows_bin;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|current target number of points: " << _target_number_of_keypoints << std::endl)

  //ds compute target points per detector region
  _target_number_of_keypoints_per_detector = static_cast<real>(_target_number_of_keypoints)/_number_of_detectors;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|current target number of points per image region: " << _target_number_of_keypoints_per_detector << std::endl)

  //ds allocate and initialize bin grid
  _bin_map_left = new FramePoint**[_number_of_rows_bin];
  for (Index row = 0; row < _number_of_rows_bin; ++row) {
    _bin_map_left[row] = new FramePoint*[_number_of_cols_bin];
    for (Index col = 0; col < _number_of_cols_bin; ++col) {
      _bin_map_left[row][col] = nullptr;
    }
  }
  LOG_INFO(std::cerr << "BaseTracker::configure|number of horizontal bins: " << _number_of_cols_bin << " size: " << _parameters->bin_size_pixels << std::endl)
  LOG_INFO(std::cerr << "BaseTracker::configure|number of vertical bins: " << _number_of_rows_bin << " size: " << _parameters->bin_size_pixels << std::endl)

  //ds clear buffers
  _keypoints_with_descriptors_left.clear();
  LOG_INFO(std::cerr << "BaseFramePointGenerator::configure|configured" << std::endl)
}

BaseFramePointGenerator::~BaseFramePointGenerator() {
  LOG_INFO(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroying" << std::endl)

  //ds deallocate dynamic data structures: detectors
  // if (_detector_regions && _detector_thresholds) {
  if (_detectors && _detector_regions && _detector_thresholds) {
    for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
      delete[] _detectors[r];
      delete[] _detector_regions[r];
      delete[] _detector_thresholds[r];
    }
    delete [] _detectors;
    delete [] _detector_regions;
    delete [] _detector_thresholds;
  }

  //ds free bin map
  for (Count row = 0; row < _number_of_rows_bin; ++row) {
    delete[] _bin_map_left[row];
  }
  delete[] _bin_map_left;
  LOG_INFO(std::cerr << "BaseFramePointGenerator::~BaseFramePointGenerator|destroyed" << std::endl)
}

void BaseFramePointGenerator::detectKeypoints(const cv::Mat& intensity_image_,
                                              std::vector<cv::KeyPoint>& keypoints_,
                                              const bool ignore_minimum_detector_threshold_) {
  CHRONOMETER_START(keypoint_detection)
  EASY_BLOCK("KeypointDetection", profiler::colors::Red);

  //ds detect new keypoints in each image region
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {

      //ds detect keypoints in current region
      std::vector<cv::KeyPoint> keypoints_per_detector(0);
      _detectors[r][c]->detect(intensity_image_(_detector_regions[r][c]), keypoints_per_detector);
      // std::cout << keypoints_per_detector.size() <<std::endl;
      std::cout <<"________________________________________________"<<std::endl;
      std::cout<<"angle:"<<keypoints_per_detector[0].angle<<std::endl;
      std::cout<<"class_id:"<<keypoints_per_detector[0].class_id<<std::endl;
      std::cout<<"octave:"<<keypoints_per_detector[0].octave<<std::endl;
      std::cout<<"response:"<<keypoints_per_detector[0].response<<std::endl;
      std::cout<<"size:"<<keypoints_per_detector[0].size<<std::endl;
      std::cout <<"________________________________________________"<<std::endl;

      //ds retrieve currently set threshold for this detector
      real detector_threshold = _detectors[r][c]->getThreshold();
      std::cout << "threshold:" << detector_threshold <<std::endl;

      //ds compute point delta: 100% loss > -1, 100% gain > +1
      const real delta = (static_cast<real>(keypoints_per_detector.size())-_target_number_of_keypoints_per_detector)/_target_number_of_keypoints_per_detector;

      //ds check if there's a significant loss of target points (delta is negative)
      if (delta < -_parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, lower threshold, capped (negative value)
        const real change = std::max(delta, -_parameters->detector_threshold_maximum_change);

        //ds always lower threshold by at least 1
        detector_threshold = detector_threshold+std::min(change*detector_threshold, -1.0);

        //ds check minimum threshold
        if (detector_threshold < _parameters->detector_threshold_minimum) {
          detector_threshold = _parameters->detector_threshold_minimum;
        }
      }

      //ds or if there's a significant gain of target points (delta is positive)
      else if (delta > _parameters->target_number_of_keypoints_tolerance) {

        //ds compute new, higher threshold, capped (positive value)
        const real change = std::min(delta, _parameters->detector_threshold_maximum_change);

        //ds always increase threshold by at least 1
        detector_threshold += std::max(change*detector_threshold, 1.0);

        //ds check maximum threshold
        if (detector_threshold > _parameters->detector_threshold_maximum) {
          detector_threshold = _parameters->detector_threshold_maximum;
        }
      }
      //ds set treshold variable (will be effectively changed by calling adjustDetectorThresholds)
      _detector_thresholds[r][c] += detector_threshold;

      //ds shift keypoint coordinates to whole image region
      const cv::Point2f& offset = _detector_regions[r][c].tl();
      std::for_each(keypoints_per_detector.begin(), keypoints_per_detector.end(), [&offset](cv::KeyPoint& keypoint_) {keypoint_.pt += offset;});

      //ds add to complete vector
      keypoints_.insert(keypoints_.end(), keypoints_per_detector.begin(), keypoints_per_detector.end());
    }
  }
  ++_number_of_detections;
  _number_of_detected_keypoints = keypoints_.size();
  EASY_END_BLOCK;
  CHRONOMETER_STOP(keypoint_detection)
}

void BaseFramePointGenerator::computeDescriptors(const cv::Mat& intensity_image_, std::vector<cv::KeyPoint>& keypoints_, cv::Mat& descriptors_) {
  CHRONOMETER_START(descriptor_extraction)
  EASY_BLOCK("DescriptorExtraction", profiler::colors::Orange);
  _descriptor_extractor->compute(intensity_image_, keypoints_, descriptors_);
  std::cout << "descriptor size" << descriptors_.size() << std::endl;
  EASY_END_BLOCK;
  CHRONOMETER_STOP(descriptor_extraction)
}

void BaseFramePointGenerator::adjustDetectorThresholds() {
  if(_parameters->detector_type == "SIFT")
    return;
  _mean_detector_threshold = 0;
  for (uint32_t r = 0; r < _parameters->number_of_detectors_vertical; ++r) {
    for (uint32_t c = 0; c < _parameters->number_of_detectors_horizontal; ++c) {
      // std::cout<<"th:"<<_detector_thresholds[r][c]<<std::endl;
      //ds compute average threshold over last detections
      _detector_thresholds[r][c] /= _number_of_detections;
      _mean_detector_threshold += _detector_thresholds[r][c];
      _detectors[r][c]->setThreshold(_detector_thresholds[r][c]);

      //ds reset bookkeeping for next detection(s)
      _detector_thresholds[r][c] = 0;
    }
  }
  _number_of_detections = 0;
  _mean_detector_threshold /= _number_of_detectors;
}

const PointCoordinates BaseFramePointGenerator::getPointInCamera(const cv::Point2f& image_point_previous_,
                                                                 const cv::Point2f& image_point_current_,
                                                                 const TransformMatrix3D& camera_previous_to_current_,
                                                                 const Matrix3& camera_calibration_matrix_) const {
  const Eigen::Matrix<real, 1, 3>& r_1 = camera_previous_to_current_.linear().block<1,3>(0,0);
  const Eigen::Matrix<real, 1, 3>& r_2 = camera_previous_to_current_.linear().block<1,3>(1,0);
  const Eigen::Matrix<real, 1, 3>& r_3 = camera_previous_to_current_.linear().block<1,3>(2,0);

  //ds obtain normalized image coordinates
  const real a_0 = (image_point_previous_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const real b_0 = (image_point_previous_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);
  const real a_1 = (image_point_current_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const real b_1 = (image_point_current_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);

  //ds initialize homogeneous coordinates in x and y
  const PointCoordinates x_0(a_0, b_0, 1);
  const PointCoordinates x_1(a_1, b_1, 1);

  //ds build constraint matrix
  Eigen::Matrix<real, 3, 2> A;
  A << -r_1*x_0, a_1,
       -r_2*x_0, b_1,
       -r_3*x_0, 1;

  //ds minimize squared error for both image points
  const Vector2 z = A.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).solve(camera_previous_to_current_.translation());

  //ds compute candidates
  const PointCoordinates point_in_camera_previous = x_0*z(0);
  const PointCoordinates point_in_camera_current  = x_1*z(1);

  //ds compute midpoint in current frame
  return (point_in_camera_current+camera_previous_to_current_*point_in_camera_previous)/2.0;
}

}
