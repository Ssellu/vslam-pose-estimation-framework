#ds export assembly as library
add_library(srrg_proslam_slam_assembly_library slam_assembly.cpp)
target_link_libraries(srrg_proslam_slam_assembly_library
  srrg_proslam_map_optimization_library
  srrg_proslam_position_tracking_library
  srrg_proslam_relocalization_library
  srrg_proslam_visualization_library
  srrg_messages_library
  srrg_system_utils_library
  ${EASY_PROFILER_LIBS}
)
