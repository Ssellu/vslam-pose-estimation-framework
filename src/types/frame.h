#pragma once
#include "parameters.h"
#include "camera.h"
#include "frame_point.h"

namespace proslam {
  
//ds forward declarations
class LocalMap;
class WorldMap;

//ds this class encapsulates all data gained from the processing of a stereo image pair
class Frame {

//ds exported types
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //ds defines one of the two tracker states
  enum Status {Localizing, Tracking};

  //ds prohibit default construction
  Frame() = delete;

//ds object handling
public: //ds TODO protect for factory

  //ds frame construction in the WorldMap context
  Frame(const WorldMap* context_,
        Frame* previous_,
        Frame* next_,
        const TransformMatrix3D& robot_to_world_,
        const double& timestamp_image_left_seconds_);

  //ds FramePoints cleanup
  ~Frame();

//ds getters/setters
public:

  const Identifier& identifier() const {return _identifier;}

  inline void setTimestampImageLeftSeconds(const double& timestamp_image_left_seconds_) {_timestamp_image_left_seconds = timestamp_image_left_seconds_;}
  const double& timestampImageLeftSeconds() const {return _timestamp_image_left_seconds;}

  inline const Frame* root() const {return _root;}
  inline void setRoot(const Frame* root_) {_root = root_;}

  inline Frame* previous() {return _previous;}
  inline const Frame* previous() const {return _previous;}
  void setPrevious(Frame* previous_) {_previous = previous_;}

  inline Frame* next() {return _next;}
  inline const Frame* next() const {return _next;}
  void setNext(Frame* next_) {_next = next_;}

  void breakTrack() {_is_track_broken = true;}
  const bool& isTrackBroken() const {return _is_track_broken;}
  void setHasReliablePoseEstimate(const bool& has_reliable_pose_estimate_) {_has_reliable_pose_estimate = has_reliable_pose_estimate_;}
  const bool& hasReliablePoseEstimate() const {return _has_reliable_pose_estimate;}

  void setProjectionTrackingDistancePixels(const uint32_t& projection_tracking_distance_pixels_) {_projection_tracking_distance_pixels = projection_tracking_distance_pixels_;}
  const uint32_t& projectionTrackingDistancePixels() const {return _projection_tracking_distance_pixels;}

  inline std::vector<cv::KeyPoint>& keypointsLeft() {return _keypoints_left;}
  inline std::vector<cv::KeyPoint>& keypointsRight() {return _keypoints_right;}
  inline cv::Mat& descriptorsLeft() {return _descriptors_left;}
  inline cv::Mat& descriptorsRight() {return _descriptors_right;}

  inline const Camera* cameraLeft() const {return _camera_left;}
  void setCameraLeft(const Camera* camera_);

  inline const Camera* cameraRight() const {return _camera_right;}
  void setCameraRight(const Camera* camera_) {_camera_right = camera_;}

  inline const TransformMatrix3D& robotToWorld() const {return _robot_to_world;}
  void setRobotToWorld(const TransformMatrix3D& robot_to_world_, const bool update_local_map_ = false);
  inline const TransformMatrix3D& worldToRobot() const {return _world_to_robot;}
  inline const TransformMatrix3D& cameraLeftToWorld() const {return _camera_left_to_world;}
  inline const TransformMatrix3D& worldToCameraLeft() const {return _world_to_camera_left;}

  inline const TransformMatrix3D& robotToLocalMap() const {return _robot_to_local_map;}
  inline const TransformMatrix3D& localMapToRobot() const {return _local_map_to_robot;}

  //ds visualization only
  void setRobotToWorldGroundTruth(const TransformMatrix3D& robot_to_world_ground_truth_) {_robot_to_world_ground_truth = robot_to_world_ground_truth_; _is_ground_truth_set = true;}
  const TransformMatrix3D& robotToWorldGroundTruth() const {return _robot_to_world_ground_truth;}

  inline const FramePointPointerVector& points() const {return _active_points;}
  inline FramePointPointerVector& points() {return _active_points;}

  //! @brief request a new framepoint generated from rigid stereo RGB input - we always know the camera coordinates
  FramePoint* createFramepoint(const IntensityFeature* feature_left_,
                               const IntensityFeature* feature_right_,
                               const real& descriptor_distance_triangulation_,
                               const PointCoordinates& camera_coordinates_left_,
                               FramePoint* previous_point_ = nullptr);

  //! @brief request a new framepoint generated from RGB-D input with a valid pixel depth - we always know the camera coordinates
  FramePoint* createFramepoint(const IntensityFeature* feature_left_,
                               const PointCoordinates& camera_coordinates_left_,
                               FramePoint* previous_point_ = nullptr);

  //! @brief request a new framepoint that has to be triangulated first before entering in the active points  without right keypoint/descriptor
  //! @brief point has NOT a valid, measured position in the camera
  FramePoint* createFramepoint(const IntensityFeature* feature_left_,
                               FramePoint* previous_point_ = nullptr);

  //! @brief created framepoints by this factory
  inline const FramePointPointerVector& createdPoints() const {return _created_points;}

  inline FramePointPointerVector& temporaryPoints() {return _temporary_points;}
  inline const FramePointPointerVector& temporaryPoints() const {return _temporary_points;}

  inline const cv::Mat& intensityImageLeft() const {return _intensity_image_left;}
  void setIntensityImageLeft(const cv::Mat intensity_image_)  {_intensity_image_left = intensity_image_;}

  inline const cv::Mat& intensityImageRight() const {return _intensity_image_right;}
  inline cv::Mat& intensityImageRight() {return _intensity_image_right;}
  void setIntensityImageRight(const cv::Mat intensity_image_)  {_intensity_image_right = intensity_image_;}
  void releaseImages() {_intensity_image_left.release(); _intensity_image_right.release();}

  inline const Status& status() const {return _status;}
  void setStatus(const Status& status_) {_status = status_;}

  void setLocalMap(LocalMap* local_map_) {_local_map = local_map_;}
  inline LocalMap* localMap() {return _local_map;}
  inline const LocalMap* localMap() const {return _local_map;}
  void setRobotToLocalMap(const TransformMatrix3D& frame_to_local_map_) {_robot_to_local_map = frame_to_local_map_; _local_map_to_robot = _robot_to_local_map.inverse();}
  void setIsKeyframe(const bool& is_keyframe_) {_is_keyframe = is_keyframe_;}
  inline const bool isKeyframe() const {return _is_keyframe;}

  //ds free all point instances
  void clear();

  //ds update framepoint world coordinates
  void updateActivePoints();

  //ds visualization only
  const bool& isGroundTruthSet() const {return _is_ground_truth_set;}

  void setAverageDescriptorDistanceTracking(const real& distance_) {_average_descriptor_distance = distance_;}
  const real& averageDescriptorDistanceTracking() const {return _average_descriptor_distance;}

  //ds reset allocated object counter
  static void reset() {_instances = 0;}

//ds attributes
protected:

  //ds unique identifier for a landmark (exists once in memory)
  const Identifier _identifier;

  //! @brief full acquisition timestamp of _intensity_image_left in seconds
  double _timestamp_image_left_seconds;

  //ds tracker status at the time of creation of this instance
  Status _status = Localizing;

  //ds links to preceding and subsequent instances
  Frame* _previous = 0;
  Frame* _next     = 0;

  //! @brief flag, set if track broke during processing this
  bool _is_track_broken =  false;

  //! @brief flag, set if pose optimization produced a reliable result (based on average chi and inliers ..)
  bool _has_reliable_pose_estimate = false;

  //! @brief pixel tracking distance used for this frame
  uint32_t _projection_tracking_distance_pixels = 0;

  //! @brief detected keypoints at the time of creation of the Frame
  std::vector<cv::KeyPoint> _keypoints_left;
  std::vector<cv::KeyPoint> _keypoints_right;

  //! @brief extracted descriptors associated to the keypoints at the time of creation of the Frame
  cv::Mat _descriptors_left;
  cv::Mat _descriptors_right;

  //! @brief mean descriptor distance for tracking
  real _average_descriptor_distance = 0;

  //! @brief bookkeeping: all created framepoints for this frame (create function)
  FramePointPointerVector _created_points;

  //! @brief bookkeeping: active (used) framepoints in the pipeline (a subset of _created_points)
  FramePointPointerVector _active_points;

  //! @brief bookkeeping: tracked framepoints that have to be estimated yet (e.g. triangulation)
  FramePointPointerVector _temporary_points;

  //ds spatials
  TransformMatrix3D _robot_to_local_map = TransformMatrix3D::Identity();
  TransformMatrix3D _local_map_to_robot = TransformMatrix3D::Identity();
  TransformMatrix3D _robot_to_world     = TransformMatrix3D::Identity();
  TransformMatrix3D _world_to_robot     = TransformMatrix3D::Identity();

  TransformMatrix3D _camera_left_to_world = TransformMatrix3D::Identity();
  TransformMatrix3D _world_to_camera_left = TransformMatrix3D::Identity();

  //ds stereo camera configuration affiliated with this frame
  const Camera* _camera_left   = 0;
  const Camera* _camera_right  = 0;

  //ds to support arbitrary number of rgb/depth image combinations
  cv::Mat _intensity_image_left;
  cv::Mat _intensity_image_right;

  //ds link to a local map if the frame is part of one
  LocalMap* _local_map;
  bool _is_keyframe  = false;

  //ds access
  friend class WorldMap;

  //ds visualization only
  TransformMatrix3D _robot_to_world_ground_truth = TransformMatrix3D::Identity();
  bool _is_ground_truth_set                      = false;
  const Frame* _root;

  //ds class specific
  private:

    //ds inner instance count - incremented upon constructor call (also unsuccessful calls)
    static Count _instances;
};

typedef std::vector<Frame*> FramePointerVector;
typedef std::pair<const Identifier, Frame*> FramePointerMapElement;
typedef std::map<const Identifier, Frame*> FramePointerMap;
}
