add_executable(LBRTorqueSineOverlay EXCLUDE_FROM_ALL LBRTorqueSineOverlayClient.h
                                                     LBRTorqueSineOverlayClient.cpp
                                                     LBRTorqueSineOverlayApp.cpp)

target_link_libraries(LBRTorqueSineOverlay FRIClient)
