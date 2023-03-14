#include "pose_tracker_3d.h"
#include "aligners/stereouv_aligner.h"

namespace proslam {
using namespace srrg_core;

PoseTracker3D::PoseTracker3D(PoseTracker3DParameters* parameters_): _parameters(parameters_) {
  LOG_INFO(std::cerr << "PoseTracker3D::PoseTracker3D|constructed" << std::endl)
}

void PoseTracker3D::configure() {
  LOG_INFO(std::cerr << "PoseTracker3D::configure|configuring" << std::endl)
  assert(_pose_optimizer);
  _previous_to_current_camera.setIdentity();
  _lost_points.clear();

  //ds initial setup: maximal tracking window with minimal descriptor distance tolerance
  _projection_tracking_distance_pixels  = _framepoint_generator->parameters()->maximum_projection_tracking_distance_pixels;
  _current_descriptor_distance_tracking = _framepoint_generator->parameters()->minimum_descriptor_distance_tracking;
  LOG_INFO(std::cerr << "PoseTracker3D::configure|configured" << std::endl)
}

//ds dynamic cleanup
PoseTracker3D::~PoseTracker3D() {
  LOG_INFO(std::cerr << "PoseTracker3D::~PoseTracker3D|destroying" << std::endl)
  _lost_points.clear();
  delete _framepoint_generator;
  delete _pose_optimizer;
  LOG_INFO(std::cerr << "PoseTracker3D::~PoseTracker3D|destroyed" << std::endl)
}

void PoseTracker3D::compute() {
  assert(_camera_left);
  assert(_context);

  //ds reset point configurations
  _number_of_tracked_points = 0;
  _context->currentlyTrackedLandmarks().clear();

  //ds check if initial guess can be refined with a motion model or other input
  switch(_parameters->motion_model) {

    //ds use camera odometry as motion guess
    case Parameters::MotionModel::CONSTANT_VELOCITY: {

      //ds nothing to do, use the previous, refined motion estimate as guess for the current
      break;
    }

    //ds use camera odometry as motion guess
    case Parameters::MotionModel::CAMERA_ODOMETRY: {

      //ds initialize guess if set
      if (_context->currentFrame()){
        _previous_to_current_camera = _camera_left_in_world_guess.inverse()*_camera_left_in_world_guess_previous;
      }
      _camera_left_in_world_guess_previous = _camera_left_in_world_guess;
      break;
    }

    //ds no guess (identity)
    default: {
      _previous_to_current_camera.setIdentity();
      break;
    }
  }

  //ds create a new frame
  Frame* current_frame = _context->createFrame();
  current_frame->setCameraLeft(_camera_left);
  current_frame->setIntensityImageLeft(_intensity_image_left);
  current_frame->setCameraRight(_camera_secondary);
  current_frame->setIntensityImageRight(_image_secondary);
  current_frame->setStatus(_status);

  //ds the new frame is automatically linked to the previous
  Frame* previous_frame = current_frame->previous();

  //ds initialize framepoint generator (specific)
  _framepoint_generator->initialize(current_frame);

  //ds if possible - attempt to track the points from the previous frame
  if (previous_frame) {

    //ds see if we're tracking by appearance (expecting large photometric displacements between images)
    //ds always do it for the first 2 frames (repeats after track is lost)
    bool track_by_appearance = (_status == Frame::Localizing);

    //ds search point tracks
    CHRONOMETER_START(tracking);
    _track(previous_frame, current_frame, track_by_appearance);
    CHRONOMETER_STOP(tracking);
  }

  //ds check previous tracker status
  switch(_status) {

    //ds localization mode - always running on tracking by appearance with maximum window size
    case Frame::Localizing: {

      //ds if we don't have a previous frame
      if (!previous_frame) {
        break;
      }

      //ds if we got not enough tracks to evaluate for position tracking
      if (_number_of_tracked_points < _parameters->minimum_number_of_landmarks_to_track) {
        _fallbackEstimate(current_frame, previous_frame);
        break;
      }

      LOG_INFO(std::cerr << "PoseTracker3D::compute|STATE: LOCALIZING|current tracks: "
                         << _number_of_tracked_points << "/" << previous_frame->points().size()
                         << " at image: " << current_frame->identifier() << std::endl)

      //ds solve pose on frame points only
      CHRONOMETER_START(pose_optimization);
      _pose_optimizer->parameters()->enable_inverse_depth_as_information = false;
      _pose_optimizer->initialize(previous_frame, current_frame, _previous_to_current_camera);
      _pose_optimizer->converge();
      CHRONOMETER_STOP(pose_optimization);

      //ds if the pose computation result is not acceptable
      if (_pose_optimizer->numberOfInliers() < _parameters->minimum_number_of_landmarks_to_track) {

        //ds fall back to secondary estimate
        _fallbackEstimate(current_frame, previous_frame);
        break;
      }

      //ds compute resulting motion
      const TransformMatrix3D& previous_to_current_camera = _pose_optimizer->previousToCurrent();
      const real delta_angular                            = WorldMap::toOrientationRodrigues(previous_to_current_camera.linear()).norm();
      const real delta_translational                      = previous_to_current_camera.translation().norm();

        //ds if the posit result is significant enough
      if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {

        //ds update motion estimate
        _previous_to_current_camera = previous_to_current_camera;

        //ds compute current robot pose
        const TransformMatrix3D camera_left_to_world = previous_frame->cameraLeftToWorld()*_previous_to_current_camera.inverse();
        current_frame->setRobotToWorld(camera_left_to_world*_camera_left->robotToCamera());
        LOG_WARNING(std::cerr << "PoseTracker3D::compute|using posit on frame points (experimental) inliers: " << _pose_optimizer->numberOfInliers()
                  << " outliers: " << _pose_optimizer->numberOfOutliers() << " average error: " << _pose_optimizer->totalError()/_pose_optimizer->numberOfInliers() <<  std::endl)
      } else {

        //ds fall back to secondary estimate
        _fallbackEstimate(current_frame, previous_frame);
      }
      break;
    }

    //ds on the track
    case Frame::Tracking: {
      _registerRecursive(previous_frame, current_frame);
      break;
    }
    default: {
      throw std::runtime_error("invalid tracker state");
      break;
    }
  }

  //ds update context pose
  _context->setRobotToWorld(current_frame->robotToWorld());

  //ds if we processed a previous frame
  if (previous_frame) {

    //ds prune current frame points
    _prunePoints(current_frame);

    //ds recover lost points based on refined pose
    if (_parameters->enable_landmark_recovery) {
      CHRONOMETER_START(point_recovery)
      _framepoint_generator->recoverPoints(current_frame, _lost_points);
      _number_of_tracked_points = current_frame->points().size();
      CHRONOMETER_STOP(point_recovery)
    }

    //ds release previous images
    previous_frame->releaseImages();
  }

  //ds trigger framepoint update and automatic landmark creation
  _updatePoints(_context, current_frame);

  //ds check if we switch to tracking state
  if (_number_of_active_landmarks > _parameters->minimum_number_of_landmarks_to_track) {
    _status = Frame::Tracking;
  }

  //ds compute remaining points in frame
  CHRONOMETER_START(track_creation)
  _framepoint_generator->compute(current_frame);
  CHRONOMETER_STOP(track_creation)
  current_frame->setStatus(_status);

  //ds update bookkeeping
  _number_of_tracked_landmarks_previous = _context->currentlyTrackedLandmarks().size();
  _total_number_of_tracked_points      += _number_of_tracked_points;

  //ds update stats
  _mean_number_of_framepoints = (_mean_number_of_framepoints*(_context->frames().size()-1)+current_frame->points().size())/_context->frames().size();
  _intensity_image_left.release();
  _image_secondary.release();
}

//ds retrieves framepoint correspondences between previous and current frame
void PoseTracker3D::_track(Frame* previous_frame_,
                           Frame* current_frame_,
                           const bool& track_by_appearance_) {

  //ds check state for current framepoint tracking configuration
  if (track_by_appearance_) {

    //ds maximimize tracking window
    _projection_tracking_distance_pixels = _framepoint_generator->parameters()->maximum_projection_tracking_distance_pixels;
  }

  //ds configure and track points in current frame
  _framepoint_generator->setProjectionTrackingDistancePixels(_projection_tracking_distance_pixels);
  _framepoint_generator->setMaximumDescriptorDistanceTracking(_current_descriptor_distance_tracking);
  _framepoint_generator->track(current_frame_, previous_frame_, _previous_to_current_camera, _lost_points, track_by_appearance_);

  //ds adjust bookkeeping
  _number_of_tracked_landmarks = _framepoint_generator->numberOfTrackedLandmarks();
  _number_of_tracked_points    = current_frame_->points().size();

  //ds compute tracking ratio (0,1)
  _tracking_ratio = static_cast<real>(_number_of_tracked_points)/previous_frame_->points().size();
  const real landmark_per_point     = static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_points;
  const real tracking_success_ratio = static_cast<real>(_number_of_tracked_points)/_framepoint_generator->targetNumberOfKeypoints();

  //ds if we're below the target - raise tracking window for next image
  if (_tracking_ratio < _parameters->good_tracking_ratio/2) {

    //ds if we still can increase the tracking window size
    if (_projection_tracking_distance_pixels < _framepoint_generator->parameters()->maximum_projection_tracking_distance_pixels) {
      _projection_tracking_distance_pixels = std::min(_projection_tracking_distance_pixels*1/_parameters->tunnel_vision_ratio,
                                                      static_cast<real>(_framepoint_generator->parameters()->maximum_projection_tracking_distance_pixels));
    }

  //ds narrow tracking window
  } else {

    //ds if we still can reduce the tracking window size
    if (_projection_tracking_distance_pixels > _framepoint_generator->parameters()->minimum_projection_tracking_distance_pixels) {
      _projection_tracking_distance_pixels = std::max(_projection_tracking_distance_pixels*_parameters->tunnel_vision_ratio,
                                                      static_cast<real>(_framepoint_generator->parameters()->minimum_projection_tracking_distance_pixels));
    }
  }

  //ds if the tracking ratio is insufficient based on several heuristics
  if (_tracking_ratio < _parameters->good_tracking_ratio ||
      _number_of_tracked_points < _pose_optimizer->parameters()->minimum_number_of_inliers ||
      (landmark_per_point < 0.5 && tracking_success_ratio < 0.25)) {

    //ds be less restrictive in tracking
    _current_descriptor_distance_tracking += 5;
    if (_current_descriptor_distance_tracking > _framepoint_generator->parameters()->maximum_descriptor_distance_tracking) {
      _current_descriptor_distance_tracking = _framepoint_generator->parameters()->maximum_descriptor_distance_tracking;
    }

  //ds if we have a sufficiently high tracking ratio
  } else {

    //ds be more restrictive in tracking
    _current_descriptor_distance_tracking -= 5;
    if (_current_descriptor_distance_tracking < _framepoint_generator->parameters()->minimum_descriptor_distance_tracking) {
      _current_descriptor_distance_tracking = _framepoint_generator->parameters()->minimum_descriptor_distance_tracking;
    }
  }

  //ds stats
  _mean_tracking_ratio = (_context->frames().size()*_mean_tracking_ratio+_tracking_ratio)/(1+_context->frames().size());

  //ds update frame with current points
  _total_number_of_landmarks += _number_of_tracked_landmarks;

  //ds VISUALIZATION ONLY
  current_frame_->setProjectionTrackingDistancePixels(_projection_tracking_distance_pixels);
}

void PoseTracker3D::_registerRecursive(Frame* previous_frame_,
                                       Frame* current_frame_,
                                       const Count& recursion_) {
  assert(_number_of_tracked_landmarks_previous != 0);

  //ds current number of tracked landmarks
  real relative_number_of_tracked_landmarks_to_previous = static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_landmarks_previous;

  //ds if we got not enough tracks to evaluate for position tracking: robustness
  if (_number_of_tracked_landmarks == 0 || relative_number_of_tracked_landmarks_to_previous < 0.1) {
    ++_number_of_recursive_registrations;

    //ds if we have recursions left (currently only two)
    if (recursion_ < 2) {

      //ds fallback to no motion model if no external info is available
      if (_parameters->motion_model != Parameters::MotionModel::CAMERA_ODOMETRY) {
        _previous_to_current_camera.setIdentity();

        //ds attempt tracking by appearance (maximum window size)
        _framepoint_generator->initialize(current_frame_, false);
        _track(previous_frame_, current_frame_, true);
        _registerRecursive(previous_frame_, current_frame_, recursion_+1);
      } else {

        //ds stick to odometry guess
        const TransformMatrix3D camera_left_to_world = previous_frame_->cameraLeftToWorld()*_previous_to_current_camera.inverse();
        current_frame_->setRobotToWorld(camera_left_to_world*_camera_left->robotToCamera());
      }
      return;
    } else {

      //ds if we have some information about the camera pose (e.g. odometry)
      if (_parameters->motion_model == Parameters::MotionModel::CAMERA_ODOMETRY) {

        //ds use fallback estimate
        _fallbackEstimate(current_frame_, previous_frame_);
      } else {

        //ds failed
        breakTrack(current_frame_);
      }
      return;
    }
  }

  //ds compute ratio between landmarks and tracked points: informative only
  const real percentage_landmarks = static_cast<real>(_number_of_tracked_landmarks)/_number_of_tracked_points;
  if (percentage_landmarks < 0.1) {
    LOG_WARNING(std::cerr << "PoseTracker3D::compute|low percentage of tracked landmarks over framepoints: " << percentage_landmarks
              << " (landmarks/framepoints: " << _number_of_tracked_landmarks << "/" << _number_of_tracked_points << ")" << std::endl)
  }

  //ds call pose solver
  CHRONOMETER_START(pose_optimization)
  _pose_optimizer->parameters()->enable_inverse_depth_as_information = true;
  _pose_optimizer->initialize(previous_frame_, current_frame_, _previous_to_current_camera);
  _pose_optimizer->converge();
  CHRONOMETER_STOP(pose_optimization)

  //ds solver deltas
  const Count number_of_inliers = _pose_optimizer->numberOfInliers();

  //ds if we have enough inliers in the pose optimization
  if (number_of_inliers > _parameters->minimum_number_of_landmarks_to_track) {

    //ds info
    if (recursion_ > 0) {
      ++_number_of_recursive_registrations;
      LOG_WARNING(std::cerr << current_frame_->identifier() << "|PoseTracker3D::_registerRecursive|recursion: " << recursion_ << "|inliers: " << number_of_inliers << std::endl)
    }

    //ds setup
    const TransformMatrix3D& previous_to_current_camera = _pose_optimizer->previousToCurrent();
    const real delta_angular       = WorldMap::toOrientationRodrigues(previous_to_current_camera.linear()).norm();
    const real delta_translational = previous_to_current_camera.translation().norm();

    //ds if the posit result is significant enough
    if (delta_angular > _parameters->minimum_delta_angular_for_movement || delta_translational > _parameters->minimum_delta_translational_for_movement) {
      _previous_to_current_camera = previous_to_current_camera;

      //ds compute current robot pose
      const TransformMatrix3D camera_left_to_world = previous_frame_->cameraLeftToWorld()*_previous_to_current_camera.inverse();
      current_frame_->setRobotToWorld(camera_left_to_world*_camera_left->robotToCamera());
    } else {

      //ds use fallback estimate
      _fallbackEstimate(current_frame_, previous_frame_);
    }
  } else {
    ++_number_of_recursive_registrations;
    LOG_WARNING(std::cerr << current_frame_->identifier() << "|PoseTracker3D::_registerRecursive|recursion: " << recursion_ << "|inliers: " << number_of_inliers << std::endl)

    //ds if we have recursions left (currently only two)
    if (recursion_ < 2) {

      //ds if we still can increase the tracking window size
      if (_projection_tracking_distance_pixels < _framepoint_generator->parameters()->maximum_projection_tracking_distance_pixels) {
        ++_projection_tracking_distance_pixels;
      }

      //ds attempt new tracking with the increased window size
      _framepoint_generator->initialize(current_frame_, false);
      _track(previous_frame_, current_frame_);
      _registerRecursive(previous_frame_, current_frame_, recursion_+1);
    } else {

      //ds if we have some information about the camera pose (e.g. odometry)
      if (_parameters->motion_model == Parameters::MotionModel::CAMERA_ODOMETRY) {

        //ds try fallback estimate
        _fallbackEstimate(current_frame_, previous_frame_);
      } else {

        //ds failed
        breakTrack(current_frame_);
      }
    }
  }
}

//! @breaks the track at the current frame
void PoseTracker3D::breakTrack(Frame* frame_) {

  //ds reset state
  LOG_WARNING(std::printf("PoseTracker3D::breakTrack|LOST TRACK at frame [%06u] with tracks: %u\n", frame_->identifier(), _number_of_tracked_points))
  _status = Frame::Localizing;

  //ds stick to previous solution - simulating a fresh start
  frame_->setRobotToWorld(frame_->previous()->robotToWorld());
  _previous_to_current_camera = TransformMatrix3D::Identity();
  _number_of_tracked_points   = 0;

  //ds reset frame in world context, triggering a restart of the pipeline
  _context->breakTrack(frame_);
}

void PoseTracker3D::_prunePoints(Frame* frame_) {
  _number_of_tracked_points = 0;

  //ds if we had a sufficient pose optimization - TODO parametrize in tracker
  if (_pose_optimizer->averageError() < _pose_optimizer->parameters()->maximum_error_kernel) {
    for (Index index_point = 0; index_point < frame_->points().size(); index_point++) {
      assert(frame_->points()[index_point]->previous());

      //ds only keep inlier points
      if (_pose_optimizer->inliers()[index_point]) {
        frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
        ++_number_of_tracked_points;
      } else {

        //ds remove track
        frame_->points()[index_point]->clear();
      }
    }
  } else {
    for (Index index_point = 0; index_point < frame_->points().size(); index_point++) {
      assert(frame_->points()[index_point]->previous());

      //ds keep all points which were not suppressed in the optimization and cap the error
      if (_pose_optimizer->errors()[index_point] != -1 &&
          _pose_optimizer->errors()[index_point] < 100*_pose_optimizer->parameters()->maximum_error_kernel) {
        frame_->points()[_number_of_tracked_points] = frame_->points()[index_point];
        ++_number_of_tracked_points;
      } else {

        //ds remove track
        frame_->points()[index_point]->clear();
      }
    }
  }
  frame_->points().resize(_number_of_tracked_points);
}

//ds updates existing or creates new landmarks for framepoints of the provided frame
void PoseTracker3D::_updatePoints(WorldMap* context_, Frame* frame_) {
  CHRONOMETER_START(landmark_optimization)

  //ds buffer current pose
  const TransformMatrix3D& robot_to_world = frame_->robotToWorld();

  //ds start landmark generation/update
  _context->currentlyTrackedLandmarks().reserve(_number_of_tracked_landmarks);
  _number_of_active_landmarks = 0;
  for (FramePoint* point: frame_->points()) {
    point->setWorldCoordinates(robot_to_world*point->robotCoordinates());

    //ds skip point if tracking and not mature enough to be a landmark
    //ds skip point if its depth was estimated (i.e. not measured) TODO enable proper triangulation to allow landmarks
    if (point->trackLength() < _parameters->minimum_track_length_for_landmark_creation || point->hasUnreliableDepth()) {
      continue;
    }
    assert(point->previous());

    //ds check if the point is linked to a landmark (we can have holes in the framepoint chain for low confidence)
    Landmark* landmark = point->origin()->landmark();

    //ds if there's no landmark yet
    if (!landmark) {

      //ds create a landmark and associate it with the current framepoint
      landmark = context_->createLandmark(point);
    }

    //ds otherwise update the landmark with the framepoint measurement
    else {

      //ds update landmark position based on current point (triggered as we linked the landmark to the point)
      landmark->update(point);
    }

    //ds lock current landmark position estimate to framepoint measurement, will be use for the subsequent frame registration
    point->setCameraCoordinatesLeftLandmark(frame_->worldToCameraLeft()*landmark->coordinates());
    ++_number_of_active_landmarks;

    //ds VISUALIZATION ONLY: add landmarks to currently visible ones
    landmark->setIsCurrentlyTracked(true);
    context_->currentlyTrackedLandmarks().push_back(landmark);
  }
  LOG_DEBUG(std::cerr << "PoseTracker3D::_updatePoints|updated landmarks: " << _number_of_active_landmarks << std::endl)

  //ds update secondary points
  Count number_of_temporary_points = 0;
  for (FramePoint* point: frame_->temporaryPoints()) {
    assert(point->previous());

    //ds compute coordinates with refined estimate
    const PointCoordinates camera_coordinates_refined = _framepoint_generator->getPointInCamera(point->previous()->keypointLeft().pt,
                                                                                                point->keypointLeft().pt,
                                                                                                _previous_to_current_camera,
                                                                                                _camera_left->cameraMatrix());

    //ds skip invalid depths
    if (camera_coordinates_refined.z() <= 0) {
      continue;
    }

    //ds set coordinates
    point->setCameraCoordinatesLeft(camera_coordinates_refined);
    point->setRobotCoordinates(_camera_left->cameraToRobot()*camera_coordinates_refined);
    point->setWorldCoordinates(robot_to_world*point->robotCoordinates());
    frame_->temporaryPoints()[number_of_temporary_points] = point;
    ++number_of_temporary_points;
  }
  frame_->temporaryPoints().resize(number_of_temporary_points);
  LOG_DEBUG(std::cerr << "PoseTracker3D::_updatePoints|updated temporary points: " << frame_->temporaryPoints().size() << std::endl)
  CHRONOMETER_STOP(landmark_optimization)
}

void PoseTracker3D::_fallbackEstimate(Frame* current_frame_,
                                      Frame* previous_frame_) {

  //ds if we have some information about the camera pose (e.g. odometry)
  if (_parameters->motion_model == Parameters::MotionModel::CAMERA_ODOMETRY) {

    //ds compute current robot pose
    const TransformMatrix3D camera_left_to_world = previous_frame_->cameraLeftToWorld()*_previous_to_current_camera.inverse();
    current_frame_->setRobotToWorld(camera_left_to_world*_camera_left->robotToCamera());
  } else {

    //ds reset state and stick to previous solution
    _previous_to_current_camera = TransformMatrix3D::Identity();
    current_frame_->setRobotToWorld(previous_frame_->robotToWorld());
  }
}
}
