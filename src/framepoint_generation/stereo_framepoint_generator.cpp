#include "stereo_framepoint_generator.h"
#include "types/landmark.h"

namespace proslam {

StereoFramePointGenerator::StereoFramePointGenerator(StereoFramePointGeneratorParameters* parameters_): BaseFramePointGenerator(parameters_),
                                                                                                        _parameters(parameters_) {
  _epipolar_search_offsets_pixel.clear();
  LOG_INFO(std::cerr << "StereoFramePointGenerator::StereoFramePointGenerator|constructed" << std::endl)
}

void StereoFramePointGenerator::configure(){
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|configuring" << std::endl)
  assert(_camera_right);

  //ds integrate configuration
  _parameters->number_of_cameras = 2;
  BaseFramePointGenerator::configure();

  //ds configure stereo triangulation parameters
  _baseline        = _camera_right->baselineHomogeneous();
  _b_x             = _baseline(0);
  _baseline_meters = -_b_x/_focal_length_pixels;
  if (_baseline_meters <= 0) {
    throw std::runtime_error("StereoFramePointGenerator::configure|invalid baseline (m): '" + std::to_string(_baseline_meters) + "' verify intrinsic camera parameters");
  }
  _f_x = _camera_right->cameraMatrix()(0,0);
  _f_y = _camera_right->cameraMatrix()(1,1);
  _c_x = _camera_right->cameraMatrix()(0,2);
  _c_y = _camera_right->cameraMatrix()(1,2);

  //ds initialize feature matcher
  _feature_matcher_right.configure(_number_of_rows_image, _number_of_cols_image);

  //ds configure epipolar search ranges (minimum 0)
  _epipolar_search_offsets_pixel.push_back(0);
  for (int32_t u = 1; u <= _parameters->maximum_epipolar_search_offset_pixels; ++u) {
    _epipolar_search_offsets_pixel.push_back(u);
    _epipolar_search_offsets_pixel.push_back(-u);
  }

  //ds info
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|baseline (m): " << _baseline_meters << std::endl)
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|number of epipolar lines considered for stereo matching: " << _epipolar_search_offsets_pixel.size() << std::endl)
  LOG_INFO(std::cerr << "StereoFramePointGenerator::configure|configured" << std::endl)
}

//ds cleanup of dynamic structures
StereoFramePointGenerator::~StereoFramePointGenerator() {
  LOG_INFO(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroying" << std::endl)
  _epipolar_search_offsets_pixel.clear();
  LOG_INFO(std::cerr << "StereoFramePointGenerator::~StereoFramePointGenerator|destroyed" << std::endl)
}

void StereoFramePointGenerator::initialize(Frame* frame_, const bool& extract_features_) {
  if (!frame_) {
    throw std::runtime_error("StereoFramePointGenerator::initialize|called with empty frame");
  }

  //ds check if a new feature extraction is desired (the frame might already be set up)
  if (extract_features_) {

    //ds detect new features - check if we have information from a previous computation
    if (frame_->previous()) {
      detectKeypoints(frame_->intensityImageLeft(), frame_->keypointsLeft(), frame_->previous()->hasReliablePoseEstimate());
      detectKeypoints(frame_->intensityImageRight(), frame_->keypointsRight(), frame_->previous()->hasReliablePoseEstimate());
    } else {
      detectKeypoints(frame_->intensityImageLeft(), frame_->keypointsLeft());
      detectKeypoints(frame_->intensityImageRight(), frame_->keypointsRight());
    }
    adjustDetectorThresholds();

    //ds extract descriptors for detected features
    computeDescriptors(frame_->intensityImageLeft(), frame_->keypointsLeft(), frame_->descriptorsLeft());
    computeDescriptors(frame_->intensityImageRight(), frame_->keypointsRight(), frame_->descriptorsRight());
    _number_of_detected_keypoints = frame_->keypointsLeft().size();
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::initialize|extracted features L: " << frame_->keypointsLeft().size()
                        << " R: " << frame_->keypointsRight().size() << std::endl)

    //ds set maximum descriptor distance for triangulation depending on on state
    if (frame_->status() == Frame::Localizing) {

      //ds be conservative while localizing
      _current_maximum_descriptor_distance_triangulation = std::min(0.1*SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS, _parameters->maximum_matching_distance_triangulation);
    } else {

      //ds adjust triangulation distance: few points > narrow window as we cannot permit a relatively high ratio of invalid triangulations
      const real ratio_available_points = std::min(static_cast<real>(_number_of_detected_keypoints)/_target_number_of_keypoints, 1.0);
      _current_maximum_descriptor_distance_triangulation = std::max(ratio_available_points*_parameters->maximum_matching_distance_triangulation, 0.1*SRRG_PROSLAM_DESCRIPTOR_SIZE_BITS);
    }
  }

  //ds initialize matchers for left and right frame
  _feature_matcher_left.setFeatures(frame_->keypointsLeft(), frame_->descriptorsLeft());
  _feature_matcher_right.setFeatures(frame_->keypointsRight(), frame_->descriptorsRight());
}

void StereoFramePointGenerator::compute(Frame* frame_) {
  CHRONOMETER_START(point_triangulation)
  if (!frame_) {
    throw std::runtime_error("StereoFramePointGenerator::compute|called with empty frame");
  }
  FramePointPointerVector& framepoints(frame_->points());
  const Count number_of_points_tracked = framepoints.size();

  //ds store already present points for optional binning
  if (_parameters->enable_keypoint_binning) {
    for (FramePoint* point: frame_->points()) {
      const Index row_bin = std::rint(static_cast<real>(point->row)/_parameters->bin_size_pixels);
      const Index col_bin = std::rint(static_cast<real>(point->col)/_parameters->bin_size_pixels);
      _bin_map_left[row_bin][col_bin] = point;
    }
  }

  //ds prepare for fast stereo matching - we temporally break the IntensityFeature.index_in_vector, which will be restored when pruning
  _feature_matcher_left.sortFeatureVector();
  _feature_matcher_right.sortFeatureVector();

  //ds new framepoints - optionally filtered in a consecutive binning
  FramePointPointerVector framepoints_new(_number_of_detected_keypoints);
  Count number_of_new_points = 0;

  //ds start stereo matching for all epipolar offsets
  for (const int32_t& epipolar_offset: _epipolar_search_offsets_pixel) {
    IntensityFeaturePointerVector& features_left(_feature_matcher_left.feature_vector);
    IntensityFeaturePointerVector& features_right(_feature_matcher_right.feature_vector);

    //ds matched features (to not consider them for the next offset search)
    std::set<uint32_t> matched_indices_left;
    std::set<uint32_t> matched_indices_right;

    //ds running variable
    uint32_t index_R = 0;

    //ds loop over all left keypoints
    for (uint32_t index_L = 0; index_L < features_left.size(); index_L++) {

      //ds if there are no more points on the right to match against - stop
      if (index_R == features_right.size()) {break;}

      //ds the right keypoints are on an lower row - skip left
      while (features_left[index_L]->row < features_right[index_R]->row+epipolar_offset) {
        index_L++; if (index_L == features_left.size()) {break;}
      }
      if (index_L == features_left.size()) {break;}
      IntensityFeature* feature_left = features_left[index_L];

      //ds the right keypoints are on an upper row - skip right
      while (feature_left->row > features_right[index_R]->row+epipolar_offset) {
        index_R++; if (index_R == features_right.size()) {break;}
      }
      if (index_R == features_right.size()) {break;}

      //ds search bookkeeping
      uint32_t index_search_R       = index_R;
      real descriptor_distance_best = _current_maximum_descriptor_distance_triangulation;
      uint32_t index_best_R         = 0;

      //ds scan epipolar line for current keypoint at idx_L - exhaustive
      while (feature_left->row == features_right[index_search_R]->row+epipolar_offset) {

        //ds invalid disparity stop condition
        if (feature_left->col-features_right[index_search_R]->col < 0) {break;}

        //ds compute descriptor distance for the stereo match candidates
        const real descriptor_distance = cv::norm(feature_left->descriptor, features_right[index_search_R]->descriptor, SRRG_PROSLAM_DESCRIPTOR_NORM);
        if(descriptor_distance < descriptor_distance_best) {
          descriptor_distance_best = descriptor_distance;
          index_best_R             = index_search_R;
        }
        index_search_R++; if (index_search_R == features_right.size()) {break;}
      }

      //ds check if something was found
      if (descriptor_distance_best < _current_maximum_descriptor_distance_triangulation) {
        IntensityFeature* feature_right = features_right[index_best_R];

        //ds skip points with insufficient stereo disparity
        if (feature_left->col-feature_right->col < _parameters->minimum_disparity_pixels) {
          continue;
        }

        //ds compute a new framepoint without track
        FramePoint* framepoint = frame_->createFramepoint(feature_left,
                                                          feature_right,
                                                          descriptor_distance_best,
                                                          getPointInLeftCamera(feature_left->keypoint.pt, feature_right->keypoint.pt));
        framepoint->setEpipolarOffset(epipolar_offset);

        //ds store point for optional binning
        if (_parameters->enable_keypoint_binning) {
          const Index row_bin = std::rint(static_cast<real>(feature_left->row)/_parameters->bin_size_pixels);
          const Index col_bin = std::rint(static_cast<real>(feature_left->col)/_parameters->bin_size_pixels);

          //ds if there is already a point in the bin
          if (_bin_map_left[row_bin][col_bin]) {
            const FramePoint* current = _bin_map_left[row_bin][col_bin];

            //ds if the point in the bin is not tracked, we prefer points with maximal disparity (= maximally accurate depth estimate)
            if (!current->previous() &&
                framepoint->disparityPixels() > current->disparityPixels() &&
                framepoint->descriptorDistanceTriangulation() <= current->descriptorDistanceTriangulation()) {

              //ds overwrite the entry
              _bin_map_left[row_bin][col_bin] = framepoint;
            }
          } else {

            //ds add a new entry
            _bin_map_left[row_bin][col_bin] = framepoint;
          }
        }

        //ds set point to buffer
        framepoints_new[number_of_new_points] = framepoint;
        ++number_of_new_points;

        //ds block further matching NOTE: that we have to put the indices of the sorted vector instead of using IntensityFeature.index_in_vector
        //ds this because the IntensityFeature.index_in_vector gets broken after the sorting and we don't want to spend time on maintaining it for this task
        matched_indices_left.insert(index_L);
        matched_indices_right.insert(index_best_R);
        _feature_matcher_left.feature_lattice[feature_left->row][feature_left->col]    = nullptr;
        _feature_matcher_right.feature_lattice[feature_right->row][feature_right->col] = nullptr;

        //ds reduce search space (this eliminates all structurally conflicting matches)
        index_R = index_best_R+1;
      }
    }

    //ds remove matched indices from candidate pools
    _feature_matcher_left.prune(matched_indices_left);
    _feature_matcher_right.prune(matched_indices_right);
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::compute|epipolar offset: " << epipolar_offset
                        << " number of unmatched features L: " << features_left.size() << " R: " << features_right.size() << std::endl)
  }
  framepoints_new.resize(number_of_new_points);
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::compute|number of new stereo points: " << number_of_new_points << "/" << _number_of_detected_keypoints << std::endl)

  //ds update framepoints
  if (_parameters->enable_keypoint_binning) {

    //ds reserve space for the best case (all points can be added)
    Count number_of_points_binned = number_of_points_tracked;
    framepoints.resize(number_of_points_tracked+number_of_new_points);

    //ds accumulate new points over bin grid
    for (Index row = 0; row < _number_of_rows_bin; ++row) {
      for (Index col = 0; col < _number_of_cols_bin; ++col) {
        if (_bin_map_left[row][col] && !_bin_map_left[row][col]->previous()) {
          framepoints[number_of_points_binned] = _bin_map_left[row][col];
          ++number_of_points_binned;
        }
        _bin_map_left[row][col] = nullptr;
      }
    }
    framepoints.resize(number_of_points_binned);
    LOG_DEBUG(std::cerr << "StereoFramePointGenerator::compute|number of new stereo points binned: " << number_of_points_binned-number_of_points_tracked << std::endl)
  } else {

    //ds add all points to frame
    framepoints.insert(framepoints.end(), framepoints_new.begin(), framepoints_new.end());
  }
  CHRONOMETER_STOP(point_triangulation)
}

void StereoFramePointGenerator::track(Frame* frame_,
                                      Frame* frame_previous_,
                                      const TransformMatrix3D& camera_left_previous_in_current_,
                                      FramePointPointerVector& lost_points_,
                                      const bool track_by_appearance_) {
  if (!frame_ || !frame_previous_) {
    throw std::runtime_error("StereoFramePointGenerator::track|called with invalid frames");
  }
  assert(frame_->points().empty());
  const Matrix3& camera_calibration_matrix = _camera_left->cameraMatrix();
  FramePointPointerVector& framepoints(frame_->points());
  FramePointPointerVector& framepoints_previous(frame_previous_->points());

  //ds allocate space, existing framepoints will be overwritten!
  framepoints.resize(framepoints_previous.size());

  //ds store points for which we couldn't find a track candidate
  lost_points_.resize(framepoints_previous.size());

  //ds tracked and triangulated features (to not consider them in the exhaustive stereo matching)
  std::set<uint32_t> matched_indices_left;
  std::set<uint32_t> matched_indices_right;
  Count number_of_tracked_points       = 0;
  Count number_of_points_lost  = 0;
  _number_of_tracked_landmarks = 0;
  real accumulated_descriptor_distance = 0;

  //ds for each previous point
  for (FramePoint* point_previous: framepoints_previous) {

    //ds transform the point into the current camera frame
    const Vector3 point_in_camera_left_prediction(camera_left_previous_in_current_*point_previous->cameraCoordinatesLeft());

    //ds project the point into the current left image plane
    const Vector3 point_in_image_left(camera_calibration_matrix*point_in_camera_left_prediction);
    const int32_t col_projection_left = point_in_image_left.x()/point_in_image_left.z();
    const int32_t row_projection_left = point_in_image_left.y()/point_in_image_left.z();

    //ds skip point if not in image plane
    if (col_projection_left < 0 || col_projection_left > _number_of_cols_image ||
        row_projection_left < 0 || row_projection_left > _number_of_rows_image) {
      continue;
    }

    //ds TRACKING obtain matching feature in left image (if any)
    real descriptor_distance_best = _maximum_descriptor_distance_tracking;

    //ds define search region (rectangular ROI)
    int32_t row_start_point = std::max(row_projection_left-_projection_tracking_distance_pixels, 0);
    int32_t row_end_point   = std::min(row_projection_left+_projection_tracking_distance_pixels+1, _number_of_rows_image);
    int32_t col_start_point = std::max(col_projection_left-_projection_tracking_distance_pixels, 0);
    int32_t col_end_point   = std::min(col_projection_left+_projection_tracking_distance_pixels+1, _number_of_cols_image);

    //ds find the best match for the previous left feature (i.e. track it)
    IntensityFeature* feature_left = _feature_matcher_left.getMatchingFeatureInRectangularRegion(row_projection_left,
                                                                                                 col_projection_left,
                                                                                                 point_previous->descriptorLeft(),
                                                                                                 row_start_point,
                                                                                                 row_end_point,
                                                                                                 col_start_point,
                                                                                                 col_end_point,
                                                                                                 _maximum_descriptor_distance_tracking,
                                                                                                 track_by_appearance_,
                                                                                                 descriptor_distance_best);

    //ds if we found a match
    if (feature_left) {

      //ds compute projection offset (i.e. prediction error > optical flow)
      const cv::Point2f projection_error(col_projection_left-feature_left->keypoint.pt.x, row_projection_left-feature_left->keypoint.pt.y);

      //ds project point into the right image - correcting by the prediction error
      const Vector3 point_in_image_right(point_in_image_left+_baseline);
      const int32_t col_projection_right_corrected = point_in_image_right.x()/point_in_image_right.z()-projection_error.x;
      const int32_t row_projection_right_corrected = point_in_image_right.y()/point_in_image_right.z()-projection_error.y;

      //ds skip point if not in image plane
      if (col_projection_right_corrected < 0 || col_projection_right_corrected > _number_of_cols_image ||
          row_projection_right_corrected < 0 || row_projection_right_corrected > _number_of_rows_image) {
        continue;
      }

      //ds TRIANGULATION: obtain matching feature in right image (if any)
      //ds we reduce the vertical matching space to the epipolar range - we search only to the left of the measure left camera coordinate
      const int32_t epipolar_offset_previous = std::fabs(point_previous->epipolarOffset());
      row_start_point = std::max(row_projection_right_corrected-epipolar_offset_previous, 0);
      row_end_point   = std::min(row_projection_right_corrected+epipolar_offset_previous+1, _number_of_rows_image);
      col_start_point = std::max(col_projection_right_corrected-_projection_tracking_distance_pixels, 0);
      col_end_point   = std::min(col_projection_right_corrected+_projection_tracking_distance_pixels+1, feature_left->col);

      //ds we might increase the matching tolerance (maximum_matching_distance_triangulation) since we have a strong prior on location
      IntensityFeature* feature_right = _feature_matcher_right.getMatchingFeatureInRectangularRegion(row_projection_right_corrected,
                                                                                                     col_projection_right_corrected,
                                                                                                     feature_left->descriptor,
                                                                                                     row_start_point,
                                                                                                     row_end_point,
                                                                                                     col_start_point,
                                                                                                     col_end_point,
                                                                                                     _current_maximum_descriptor_distance_triangulation,
                                                                                                     true,
                                                                                                     descriptor_distance_best);

      //ds if we found a match
      if (feature_right) {
        assert(feature_left->col >= feature_right->col);

        //ds skip points with insufficient stereo disparity
        if (feature_left->col-feature_right->col < _parameters->minimum_disparity_pixels) {
          continue;
        }

        //ds skip feature if descriptor distance to previous is violated
        if (cv::norm(feature_right->descriptor, point_previous->descriptorRight(), SRRG_PROSLAM_DESCRIPTOR_NORM) > _maximum_descriptor_distance_tracking) {
          continue;
        }

        //ds remove remaining matches in parallax between left and right point in the right image
        for (int32_t col = feature_right->col+1; col < feature_left->col; ++col) {
          if (_feature_matcher_right.feature_lattice[feature_right->row][col]) {
            matched_indices_right.insert(_feature_matcher_right.feature_lattice[feature_right->row][col]->index_in_vector);
            _feature_matcher_right.feature_lattice[feature_right->row][col] = nullptr;
          }
        }

        //ds create a stereo match
        FramePoint* framepoint = frame_->createFramepoint(feature_left,
                                                          feature_right,
                                                          descriptor_distance_best,
                                                          getPointInLeftCamera(feature_left->keypoint.pt, feature_right->keypoint.pt),
                                                          point_previous);
        framepoint->setEpipolarOffset(feature_right->row-feature_left->row);
        accumulated_descriptor_distance += descriptor_distance_best;

        //ds VSUALIZATION ONLY
        framepoint->setProjectionEstimateLeft(cv::Point2f(col_projection_left, row_projection_left));
        framepoint->setProjectionEstimateRight(cv::Point2f(point_in_image_right.x()/point_in_image_right.z(), point_in_image_right.y()/point_in_image_right.z()));
        framepoint->setProjectionEstimateRightCorrected(cv::Point2f(col_projection_right_corrected, row_projection_right_corrected));

        //ds store and move to next slot
        framepoints[number_of_tracked_points] = framepoint;
        ++number_of_tracked_points;

        //ds block matching in exhaustive matching (later)
        matched_indices_left.insert(feature_left->index_in_vector);
        matched_indices_right.insert(feature_right->index_in_vector);
        _feature_matcher_left.feature_lattice[feature_left->row][feature_left->col]    = nullptr;
        _feature_matcher_right.feature_lattice[feature_right->row][feature_right->col] = nullptr;

        if (point_previous->landmark()) {
          ++_number_of_tracked_landmarks;
        }
      }
    }

    //ds if we couldn't track the point
    if (!point_previous->next()) {
      lost_points_[number_of_points_lost] = point_previous;
      ++number_of_points_lost;
    }
  }
  framepoints.resize(number_of_tracked_points);
  lost_points_.resize(number_of_points_lost);
  frame_previous_->setAverageDescriptorDistanceTracking(accumulated_descriptor_distance/number_of_tracked_points);

  //ds remove matched indices from candidate pools
  _feature_matcher_left.prune(matched_indices_left);
  _feature_matcher_right.prune(matched_indices_right);
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::track|tracked and triangulated points: " << number_of_tracked_points
                      << "/" << framepoints_previous.size() << " (landmarks: " << _number_of_tracked_landmarks << ")" << std::endl)
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::track|lost points: " << number_of_points_lost
                      << "/" << framepoints_previous.size() << std::endl)
}

void StereoFramePointGenerator::recoverPoints(Frame* current_frame_, const FramePointPointerVector& lost_points_) const {

  //ds precompute transforms
  const TransformMatrix3D world_to_camera_left  = current_frame_->worldToCameraLeft();
  const CameraMatrix& camera_calibration_matrix = _camera_left->cameraMatrix();
  const Vector3 baseline_homogeneous            = _camera_right->baselineHomogeneous();

  //ds buffers
  const cv::Mat& intensity_image_left  = current_frame_->intensityImageLeft();
  const cv::Mat& intensity_image_right = current_frame_->intensityImageRight();
  std::vector<cv::KeyPoint> keypoint_buffer_left(1);
  std::vector<cv::KeyPoint> keypoint_buffer_right(1);

  //ds recover lost landmarks
  const Count number_of_tracked_points = current_frame_->points().size();
  Index index_lost_point_recovered     = number_of_tracked_points;
  current_frame_->points().resize(number_of_tracked_points+lost_points_.size());
  for (FramePoint* point_previous: lost_points_) {

    //ds only recover landmarks (TODO parametrize)
    if (!point_previous->landmark()) {
      continue;
    }

    //ds get point into current camera - based on last track
    PointCoordinates point_in_camera_homogeneous(Vector3::Zero());

    //ds if we have a landmark at hand
    if (point_previous->landmark()) {
      point_previous->landmark()->incrementNumberOfRecoveries();

      //ds get point in camera frame based on landmark coordinates
      point_in_camera_homogeneous = world_to_camera_left*point_previous->landmark()->coordinates();
    } else {

      //ds get point in camera frame based on point coordinates
      point_in_camera_homogeneous = world_to_camera_left*point_previous->worldCoordinates();
    }

    //ds obtain point projection on camera image plane
    PointCoordinates point_in_image_left  = camera_calibration_matrix*point_in_camera_homogeneous;
    PointCoordinates point_in_image_right = point_in_image_left+baseline_homogeneous;

    //ds skip invalid depths
    if (point_in_image_left.z() < _parameters->minimum_depth_meters || point_in_image_left.z() > _parameters->maximum_depth_meters  ||
        point_in_image_right.z() < _parameters->minimum_depth_meters || point_in_image_right.z() > _parameters->maximum_depth_meters) {
      continue;
    }

    //ds normalize point and update prediction based on landmark position
    point_in_image_left  /= point_in_image_left.z();
    point_in_image_right /= point_in_image_right.z();

    //ds set projections - at pixel accuarcy
    const cv::Point2f projection_left(std::rint(point_in_image_left.x()), std::rint(point_in_image_left.y()));
    const cv::Point2f projection_right(std::rint(point_in_image_right.x()), std::rint(point_in_image_right.y()));

    //ds check if projection range is insufficient for recovery
    const float regional_border_center = 5*point_previous->keypointLeft().size;
    if (projection_left.x < regional_border_center+1 || projection_left.x > _camera_left->numberOfImageCols()-regional_border_center-1  ||
        projection_right.x < regional_border_center+1 || projection_right.x > _camera_left->numberOfImageCols()-regional_border_center-1||
        projection_left.y < regional_border_center+1 || projection_left.y > _camera_left->numberOfImageRows()-regional_border_center-1  ||
        projection_right.y < regional_border_center+1 || projection_right.y > _camera_left->numberOfImageRows()-regional_border_center-1) {
      continue;
    }

    //ds this can be moved outside of the loop if keypoint sizes are constant
    const cv::Point2f offset_keypoint_half(regional_border_center, regional_border_center);
    const float regional_full_height = regional_border_center+regional_border_center+1;

    //ds left search regions
    const cv::Point2f corner_left(projection_left-offset_keypoint_half);
    const cv::Rect_<float> region_of_interest_left(corner_left.x, corner_left.y, regional_full_height, regional_full_height);

    //ds extract descriptors at this position: LEFT
    keypoint_buffer_left[0]    = point_previous->keypointLeft();
    keypoint_buffer_left[0].pt = offset_keypoint_half;
    cv::Mat descriptor_left;
    const cv::Mat roi_left(intensity_image_left(region_of_interest_left));
    _descriptor_extractor->compute(roi_left, keypoint_buffer_left, descriptor_left);

    //ds if no descriptor could be computed
    if (descriptor_left.rows == 0) {
      continue;
    }

    //ds move keypoint to global image coordinates
    keypoint_buffer_left[0].pt += corner_left;

    //ds if descriptor distance is to high
    if (cv::norm(point_previous->descriptorLeft(), descriptor_left, SRRG_PROSLAM_DESCRIPTOR_NORM) > _maximum_descriptor_distance_tracking) {
      continue;
    }

    //ds right search region
    const cv::Point2f corner_right(projection_right-offset_keypoint_half);
    const cv::Rect_<float> region_of_interest_right(corner_right.x, corner_right.y, regional_full_height, regional_full_height);

    //ds extract descriptors at this position: RIGHT
    keypoint_buffer_right[0]    = point_previous->keypointRight();
    keypoint_buffer_right[0].pt = offset_keypoint_half;
    cv::Mat descriptor_right;
    const cv::Mat roi_right(intensity_image_right(region_of_interest_right));
    _descriptor_extractor->compute(roi_right, keypoint_buffer_right, descriptor_right);

    //ds if no descriptor could be computed
    if (descriptor_right.rows == 0) {
      continue;
    }

    //ds move keypoint to global image coordinates
    keypoint_buffer_right[0].pt += corner_right;

    //ds skip points with insufficient stereo disparity
    if (keypoint_buffer_left[0].pt.x-keypoint_buffer_right[0].pt.x < _parameters->minimum_disparity_pixels) {
      continue;
    }

    //ds if descriptor distance is to high
    if (cv::norm(point_previous->descriptorRight(), descriptor_right, SRRG_PROSLAM_DESCRIPTOR_NORM) > _maximum_descriptor_distance_tracking) {
      continue;
    }

    //ds check stereo triangulation distance
    const real descriptor_distance_triangulation = cv::norm(descriptor_left, descriptor_right, SRRG_PROSLAM_DESCRIPTOR_NORM);
    if (descriptor_distance_triangulation > _current_maximum_descriptor_distance_triangulation) {
      continue;
    }

    //ds instantiate features (will be owned by the framepoint)
    IntensityFeature* feature_left  = new IntensityFeature(keypoint_buffer_left[0], descriptor_left, 0);
    IntensityFeature* feature_right = new IntensityFeature(keypoint_buffer_right[0], descriptor_right, 0);

    //ds allocate a new point connected to the previous one
    FramePoint* current_point = current_frame_->createFramepoint(feature_left,
                                                                 feature_right,
                                                                 descriptor_distance_triangulation,
                                                                 getPointInLeftCamera(keypoint_buffer_left[0].pt, keypoint_buffer_right[0].pt),
                                                                 point_previous);

    delete feature_left;
    delete feature_right;

    //ds set the point to the control structure
    current_frame_->points()[index_lost_point_recovered] = current_point;
    ++index_lost_point_recovered;
  }
  current_frame_->points().resize(index_lost_point_recovered);
  LOG_DEBUG(std::cerr << "StereoFramePointGenerator::recoverPoints|recovered points: "
                      << current_frame_->points().size()-number_of_tracked_points << "/" << lost_points_.size() << std::endl)
}

const PointCoordinates StereoFramePointGenerator::getPointInLeftCamera(const cv::Point2f& image_coordinates_left_, const cv::Point2f& image_coordinates_right_) const {
  assert(image_coordinates_left_.x >= image_coordinates_right_.x);
  assert(image_coordinates_left_.x-image_coordinates_right_.x >= _parameters->minimum_disparity_pixels);

  //ds point coordinates in camera frame
  PointCoordinates position_in_left_camera(Eigen::Vector3d::Zero());

  //ds triangulate point (assuming non-zero disparity)
  position_in_left_camera.z() = _b_x/(image_coordinates_right_.x-image_coordinates_left_.x);

  //ds compute x in camera (regular)
  position_in_left_camera.x() = 1/_f_x*(image_coordinates_left_.x-_c_x)*position_in_left_camera.z();

  //ds average in case we have an epipolar offset in v
  position_in_left_camera.y() = 1/_f_y*((image_coordinates_left_.y+image_coordinates_right_.y)/2.0-_c_y)*position_in_left_camera.z();
  return position_in_left_camera;
}
}
