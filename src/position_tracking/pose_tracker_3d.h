#pragma once
#include "framepoint_generation/base_framepoint_generator.h"
#include "aligners/base_frame_aligner.h"
#include "types/world_map.h"

namespace proslam {

//ds this class processes two subsequent Frames and establishes Framepoint correspondences (tracks) based on the corresponding images
class PoseTracker3D {

//ds object handling
PROSLAM_MAKE_PROCESSING_CLASS(PoseTracker3D)

//ds functionality
public:

  //! @brief creates a new Frame for the given images, retrieves the correspondences relative to the previous Frame, optimizes the current frame pose and updates landmarks
  void compute();

  //! @breaks the track at the current frame
  //! @param[in] frame_ target frame to break the track at
  void breakTrack(Frame* frame_);

//ds getters/setters
public:

  const Count& numberOfRecursiveRegistrations() const {return _number_of_recursive_registrations;}
  void setCameraLeft(const Camera* camera_) {_camera_left = camera_;}
  void setCameraLeftInWorldGuess(const TransformMatrix3D& camera_left_in_world_guess_) {_camera_left_in_world_guess = camera_left_in_world_guess_; _has_guess = true;}
  void setCameraSecondary(const Camera* camera_) {_camera_secondary = camera_;}
  void setAligner(BaseFrameAligner* pose_optimizer_) {_pose_optimizer = pose_optimizer_;}
  void setFramePointGenerator(BaseFramePointGenerator * framepoint_generator_) {_framepoint_generator = framepoint_generator_;}
  void setWorldMap(WorldMap* context_) {_context = context_;}
  void setIntensityImageLeft(const cv::Mat& image_) {_intensity_image_left = image_;}
  void setImageSecondary(const cv::Mat& image_) {_image_secondary = image_;}
  BaseFrameAligner* aligner() {return _pose_optimizer;}
  void setMotionPreviousToCurrent(const TransformMatrix3D& motion_previous_to_current_) {_previous_to_current_camera = motion_previous_to_current_;}
  BaseFramePointGenerator* framepointGenerator() {return _framepoint_generator;}
  const BaseFramePointGenerator* framepointGenerator() const {return _framepoint_generator;}
  const Count totalNumberOfTrackedPoints() const {return _total_number_of_tracked_points;}
  const Count totalNumberOfLandmarks() const {return _total_number_of_landmarks;}
  const real meanTrackingRatio() const {return _mean_tracking_ratio;}
  const real meanNumberOfFramepoints() const {return _mean_number_of_framepoints;}

//ds helpers
protected:

  //ds retrieves framepoint correspondences between previous and current frame
  void _track(Frame* previous_frame_,
              Frame* current_frame_,
              const bool& track_by_appearance_ = false);

  //! @brief recursive registration method, that calls track framepoints with different parameters upon failure
  //! @param [in] previous_frame_ the previous frame
  //! @param [in] current_frame_ the current frame to align against the previous frame
  void _registerRecursive(Frame* previous_frame_,
                          Frame* current_frame_,
                          const Count& recursion_ = 0);

  //ds prunes invalid tracks after pose optimization
  void _prunePoints(Frame* frame_);

  //ds updates existing or creates new landmarks for framepoints of the provided frame
  void _updatePoints(WorldMap* context_, Frame* frame_);

  //! @brief resets the pose estimate to a fallback estimate
  //! depending on the selected motion model and/or additinal sensors (e.g. odometry)
  void _fallbackEstimate(Frame* current_frame_,
                         Frame* previous_frame_);

//ds attributes
protected:

  //ds current tracker state
  Frame::Status _status = Frame::Localizing;

  //ds running variables and buffered values
  Count _number_of_tracked_landmarks          = 0;
  Count _number_of_tracked_points             = 0;
  Count _number_of_tracked_landmarks_previous = 0;
  Count _number_of_active_landmarks           = 0;
  real _tracking_ratio                        = 0;
  real _mean_tracking_ratio                   = 0;
  const Camera* _camera_left                  = nullptr;
  const Camera* _camera_secondary             = nullptr;

  //! @brief currently active projection tracking distance (adjusted dynamically at runtime)
  int32_t _projection_tracking_distance_pixels = 0;
  real _current_descriptor_distance_tracking   = 0;

  //gg working elements
  cv::Mat _intensity_image_left;
  cv::Mat _image_secondary;
  WorldMap* _context = nullptr;

  //gg processing objects
  BaseFrameAligner* _pose_optimizer              = nullptr;
  BaseFramePointGenerator* _framepoint_generator = nullptr;

  //! @brief position tracking bookkeeping: after optimization
  TransformMatrix3D _previous_to_current_camera = TransformMatrix3D::Identity();

  //ds additional information
  TransformMatrix3D _camera_left_in_world_guess;
  TransformMatrix3D _camera_left_in_world_guess_previous;
  bool _has_guess = false;

  //ds track recovery
  FramePointPointerVector _lost_points;

  //ds stats only
  Count _number_of_recursive_registrations = 0;
  real _mean_number_of_framepoints = 0;

private:

  //ds informative only
  CREATE_CHRONOMETER(tracking)
  CREATE_CHRONOMETER(track_creation)
  CREATE_CHRONOMETER(pose_optimization)
  CREATE_CHRONOMETER(landmark_optimization)
  CREATE_CHRONOMETER(point_recovery)
  Count _total_number_of_tracked_points = 0;
  Count _total_number_of_landmarks      = 0;
};

typedef std::shared_ptr<PoseTracker3D> PoseTracker3DPtr;

}
