add_executable(SimulatedTransformationProvider EXCLUDE_FROM_ALL SimulatedTransformationProviderClient.h
                                                                SimulatedTransformationProviderClient.cpp
                                                                SimulatedTransformationProviderApp.cpp)

target_link_libraries(SimulatedTransformationProvider FRIClient)
