# Traverse subdirectories.
add_subdirectory(IOAccess)
add_subdirectory(LBRClientTemplate)
add_subdirectory(LBRJointSineOverlay)
add_subdirectory(LBRTorqueSineOverlay)
add_subdirectory(LBRWrenchSineOverlay)
add_subdirectory(Monitor)
add_subdirectory(SimulatedTransformationProvider)
add_subdirectory(TransformationProvider)

# Group example targets.
add_custom_target(examples)

# Register examples in group target.
add_dependencies(examples IOAccess
                          LBRClientTemplate
                          LBRJointSineOverlay
                          LBRTorqueSineOverlay
                          LBRWrenchSineOverlay
                          Monitor
                          SimulatedTransformationProvider
                          TransformationProvider)
