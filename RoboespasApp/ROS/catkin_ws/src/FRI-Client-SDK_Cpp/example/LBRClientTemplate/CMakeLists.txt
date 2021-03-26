add_executable(LBRClientTemplate EXCLUDE_FROM_ALL MyLBRClient.h
                                                  MyLBRClient.cpp
                                                  MyLBRApp.cpp)

target_link_libraries(LBRClientTemplate FRIClient)
