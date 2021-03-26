add_executable(Monitor EXCLUDE_FROM_ALL MonitorClient.h
                                        MonitorClient.cpp
                                        MonitorApp.cpp)

target_link_libraries(Monitor FRIClient)
