add_library(srrg_proslam_framepoint_generation_library
  intensity_feature_matcher.cpp
  base_framepoint_generator.cpp
  stereo_framepoint_generator.cpp
  depth_framepoint_generator.cpp
)

target_link_libraries(srrg_proslam_framepoint_generation_library
  srrg_proslam_types_library
  easy_profiler
)
