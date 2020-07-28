add_executable(LBRJointSineOverlay EXCLUDE_FROM_ALL LBRJointSineOverlayClient.h
                                                    LBRJointSineOverlayClient.cpp
                                                    LBRJointSineOverlayApp.cpp)

target_link_libraries(LBRJointSineOverlay FRIClient)
