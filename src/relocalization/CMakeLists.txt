add_library(srrg_proslam_relocalization_library
  relocalizer.cpp
)

target_link_libraries(srrg_proslam_relocalization_library
  srrg_proslam_aligners_library
  easy_profiler
)
