add_library(srrg_proslam_aligners_library
  stereouv_aligner.cpp
  uvd_aligner.cpp
  xyz_aligner.cpp
  # Add - Backend
  fast_aligner.cpp
)

target_link_libraries(srrg_proslam_aligners_library
  srrg_proslam_types_library
)
