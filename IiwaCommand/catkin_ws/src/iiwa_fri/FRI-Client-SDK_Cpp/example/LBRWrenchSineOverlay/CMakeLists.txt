add_executable(LBRWrenchSineOverlay EXCLUDE_FROM_ALL LBRWrenchSineOverlayClient.h
                                                     LBRWrenchSineOverlayClient.cpp
                                                     LBRWrenchSineOverlayApp.cpp)

target_link_libraries(LBRWrenchSineOverlay FRIClient)
