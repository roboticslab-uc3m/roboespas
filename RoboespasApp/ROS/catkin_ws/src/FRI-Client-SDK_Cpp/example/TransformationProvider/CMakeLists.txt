add_executable(TransformationProvider EXCLUDE_FROM_ALL TransformationProviderClient.h
                                                       TransformationProviderClient.cpp
                                                       TransformationProviderApp.cpp)

target_link_libraries(TransformationProvider FRIClient)
