#include "map_viewer.h"

#include "srrg_gl_helpers/opengl_primitives.h"

namespace proslam {

using namespace srrg_gl_helpers;
using namespace srrg_core_viewers;

MapViewer::MapViewer(MapViewerParameters* parameters_): _parameters(parameters_),
                                                        _world_map(0),
                                                        _current_frame(0),
                                                        _option_stepwise_playback(true),
                                                        _requested_playback_steps(0),
                                                        _camera_left_to_robot(TransformMatrix3D::Identity()),
                                                        _world_to_robot_origin(TransformMatrix3D::Identity()),
                                                        _robot_viewpoint(TransformMatrix3D::Identity()),
                                                        _world_to_robot(TransformMatrix3D::Identity()) {
  setWindowTitle(_parameters->window_title.c_str());
//  setFPSIsDisplayed(true);
  _visible_landmarks.clear();

  //ds set keyboard descriptions
  setKeyDescription(Qt::Key_1, "Toggles map points display");
  setKeyDescription(Qt::Key_2, "Toggles trajectory display at maximum granularity");
  setKeyDescription(Qt::Key_3, "Switches view to robot ego perspective");
  setKeyDescription(Qt::Key_4, "Toggles robot ground truth trajectory (red)");
  setKeyDescription(Qt::Key_5, "Decreases camera size by factor 2");
  setKeyDescription(Qt::Key_6, "Increases camera size by factor 2");
  setKeyDescription(Qt::Key_7, "Decreases point size by factor 2");
  setKeyDescription(Qt::Key_8, "Increases point size by factor 2");
  setKeyDescription(Qt::Key_Space, "Toggles stepwise/benchmark mode");

  LOG_INFO(std::cerr << DOUBLE_BAR << std::endl)
  LOG_INFO(std::cerr << "MapViewer::MapViewer|switched to stepwise mode (press [Space] for switch, press [ARROW_UP] for stepping)" << std::endl)
  LOG_INFO(std::cerr << DOUBLE_BAR << std::endl)
  LOG_DEBUG(std::cerr << "MapViewer::MapViewer|constructed" << std::endl)
}

MapViewer::~MapViewer() {
  LOG_DEBUG(std::cerr << "MapViewer::~MapViewer|destroying" << std::endl)
  _visible_landmarks.clear();
  LOG_DEBUG(std::cerr << "MapViewer::~MapViewer|destroyed" << std::endl)
}

void MapViewer::configure() {
  //ds nothing to do
}

void MapViewer::update(const Frame* frame_) {

  //ds start drawing - during that we cannot exchange the container content
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds update the current frame
  _current_frame = frame_;

  //ds update visible landmarks
  update(_world_map->currentlyTrackedLandmarks());
}

void MapViewer::update(const LandmarkPointerVector visible_landmarks_) {
  _visible_landmarks.resize(visible_landmarks_.size());
  for (Index u = 0; u < visible_landmarks_.size(); ++u) {
    _visible_landmarks[u] = visible_landmarks_[u];
  }
}

void MapViewer::_drawFrame(const Frame* frame_, const Vector3& color_rgb_) const {

  //ds check if the frame is closed and if so highlight it accordingly
  if (frame_->isKeyframe() && !frame_->localMap()->closures().empty()) {
    glColor3f(0.0, 1.0, 0.0);
    const Vector3& closure_coordinates(frame_->robotToWorld().translation());

    //ds draw closure lines
    glBegin(GL_LINES);
    for (const Closure::ClosureConstraint& constraint: frame_->localMap()->closures()) {
      const Vector3& closure_coordinates_reference(constraint.local_map->robotToWorld().translation());
      glVertex3f(closure_coordinates.x(), closure_coordinates.y(), closure_coordinates.z());
      glVertex3f(closure_coordinates_reference.x(), closure_coordinates_reference.y(), closure_coordinates_reference.z());
    }
    glEnd();
  } else {

    //ds default color
    glColor3f(color_rgb_.x(), color_rgb_.y(), color_rgb_.z());
  }

  //ds draw camera box
  glPushMatrix();
  glMultMatrixf((frame_->robotToWorld()*_camera_left_to_robot).cast<float>().data());
  drawPyramidWireframe(_parameters->object_scale, _parameters->object_scale);
  glPopMatrix();

  //ds draw pure odometry pose in red
  if (_parameters->ground_truth_drawn && frame_->isGroundTruthSet()) {
    const TransformMatrix3D camera_to_world_ground_truth = frame_->robotToWorldGroundTruth()*_camera_left_to_robot;
    glPushMatrix();
    glMultMatrixf(camera_to_world_ground_truth.cast<float>().data());
    glColor3f(1, 0, 0);
    drawPyramidWireframe(_parameters->object_scale, _parameters->object_scale);
    glPopMatrix();
  }
}

void MapViewer::_drawLandmarks() const {
  glBegin(GL_POINTS);

  //ds highlight the currently seen landmarks
  glColor3f(0, 0, 1);
  for (const Landmark* landmark: _visible_landmarks) {
    glVertex3f(landmark->coordinates().x(), landmark->coordinates().y(), landmark->coordinates().z());
  }

  //ds draw permanent landmarks
  for (const LandmarkPointerMapElement& landmark: _world_map->landmarks()) {
    const Vector3& landmark_coordinates(landmark.second->coordinates());

    //ds specific coloring for closure landmarks
    if (landmark.second->isInLoopClosureQuery()) {
      glColor3f(0, 1.0, 0);
    } else if (landmark.second->isInLoopClosureReference()) {
      glColor3f(0, 0.5, 0);
    } else {
      glColor3f(0.5, 0.5, 0.5);
    }
    glVertex3f(landmark_coordinates.x(), landmark_coordinates.y(), landmark_coordinates.z());
  }
  glEnd();

  glBegin(GL_LINES);
  const PointCoordinates& robot_in_world = _world_map->robotToWorld().translation();
  for (const Landmark* point: _visible_landmarks) {

    //ds draw old landmarks with a darker color
    const real intensity = 0.9-std::min(point->numberOfUpdates()/100.0, 0.9);
    glColor3f(intensity, intensity, intensity);
    glVertex3f(robot_in_world.x(), robot_in_world.y(), robot_in_world.z());
    glVertex3f(point->coordinates().x(), point->coordinates().y(), point->coordinates().z());
  }
  glEnd();
}

void MapViewer::_drawFramepoints() const {
  if (!_current_frame) {
    return;
  }

  glBegin(GL_POINTS);
  for (const FramePoint* point: _current_frame->points()) {
    glColor3f(0.75, 0.75, 0.75);
    glVertex3f(point->worldCoordinates().x(), point->worldCoordinates().y(), point->worldCoordinates().z());
  }
  glEnd();
}

void MapViewer::draw(){

  //ds start drawing - during that we cannot exchange the container content
  std::lock_guard<std::mutex> lock(_mutex_data_exchange);

  //ds if we got a valid handle
  if (_world_map) {

    //ds no specific lighting
    glDisable(GL_LIGHTING);

    //ds disable blending (e.g. transparency)
    glDisable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //ds set viewpoint
    TransformMatrix3D world_to_robot(_world_to_robot_origin);
    if(_parameters->follow_robot) {

      //ds check if we can get a position update (previous frame is only touched in a map update phase - thread-safe otherwise)
      if (_current_frame) {
        _world_to_robot = _current_frame->worldToRobot();
      }

      //ds set ego perspective head
      world_to_robot = _robot_viewpoint*_world_to_robot;
    } else {

      //ds draw world origin XYZ axes
      drawAxis(10*_parameters->object_scale);
    }
    glPushMatrix();
    glMultMatrixf(world_to_robot.cast<float>().data());
    glPointSize(_parameters->point_size);
    glLineWidth(_parameters->object_scale);

    //ds draw the local map generating head
    for (const Frame* frame_for_local_map: _world_map->frameQueueForLocalMap()) {
      _drawFrame(frame_for_local_map, Vector3(0, 0, 1));
    }

    //ds for all frames in the map - obtain the current root
    for (const FramePointerMapElement& frame_element: _world_map->frames()) {

      //ds check if we have a keyframe and drawing is enabled
      if (frame_element.second->isKeyframe()) {
        _drawFrame(frame_element.second, Vector3(0.5, 0.5, 1));
      } else if (_parameters->frames_drawn) {
        _drawFrame(frame_element.second, Vector3(0.75, 0.75, 1));
      }
    }

    //ds if desired, draw landmarks into map
    if (_parameters->landmarks_drawn) {
      _drawLandmarks();

      //ds also draw the framepoints of the current frame
      _drawFramepoints();
    }
    glPopMatrix();
  }
}

void MapViewer::keyPressEvent(QKeyEvent* event_){
  switch (event_->key()) {
    case Qt::Key_1: {
      if(_parameters->landmarks_drawn) {
        _parameters->landmarks_drawn = false;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|landmarks drawn - DISABLED" << std::endl)
      }
      else {
        _parameters->landmarks_drawn = true;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|landmarks drawn - ENABLED" << std::endl)
      }
      break;
    }
    case Qt::Key_2: {
      if(_parameters->frames_drawn) {
        _parameters->frames_drawn = false;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|all frames drawn - DISABLED" << std::endl)
      }
      else {
        _parameters->frames_drawn = true;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|all frames drawn - ENABLED" << std::endl)
      }
      break;
    }
    case Qt::Key_3: {
      if(_parameters->follow_robot) {
        _parameters->follow_robot = false;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|follow robot - DISABLED" << std::endl)
      }
      else {
        _parameters->follow_robot = true;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|follow robot - ENABLED" << std::endl)
      }
      break;
    }
    case Qt::Key_4: {
      if(_parameters->ground_truth_drawn) {
        _parameters->ground_truth_drawn = false;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|draw ground truth - DISABLED" << std::endl)
      }
      else {
        _parameters->ground_truth_drawn = true;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|draw ground truth - ENABLED" << std::endl)
      }
      break;
    }
    case Qt::Key_5: {
      _parameters->object_scale /= 2;
      LOG_INFO(std::cerr << "MapViewer::keyPressEvent|decreasing object scale to: " << _parameters->object_scale << std::endl)
      break;
    }
    case Qt::Key_6: {
      _parameters->object_scale *= 2;
      LOG_INFO(std::cerr << "MapViewer::keyPressEvent|increasing object scale to: " << _parameters->object_scale << std::endl)
      break;
    }
    case Qt::Key_7: {
      _parameters->point_size /= 2;
      LOG_INFO(std::cerr << "MapViewer::keyPressEvent|decreasing point size to: " << _parameters->point_size << std::endl)

      break;
    }
    case Qt::Key_8: {
      _parameters->point_size *= 2;
      LOG_INFO(std::cerr << "MapViewer::keyPressEvent|increasing point size to: " << _parameters->point_size << std::endl)
      break;
    }
    case Qt::Key_Space: {
      _option_stepwise_playback = !_option_stepwise_playback;

      //ds if we switched back to benchmark - reset the steps
      if (!_option_stepwise_playback) {
        _requested_playback_steps = 0;
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|switched to benchmark mode" << std::endl)
      } else {
        LOG_INFO(std::cerr << "MapViewer::keyPressEvent|switched to stepwise mode, press [ARROW_UP] to step" << std::endl)
      }
      break;
    }
    case Qt::Key_Up: {
      if (_option_stepwise_playback) {
        ++_requested_playback_steps;
      }
      break;
    }
    case Qt::Key_Down: {
      break;
    }
    case Qt::Key_Left: {
      break;
    }
    case Qt::Key_Right: {
      break;
    }
    default: {
      SimpleViewer::keyPressEvent(event_);
      break;
    }
  }
  draw();
  updateGL();
}

QString MapViewer::helpString( ) const {
    return "See 'Keyboard' tab for controls";
}
}
