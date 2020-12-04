add_executable(IOAccess EXCLUDE_FROM_ALL IOAccessClient.h
                                         IOAccessClient.cpp
                                         IOAccessApp.cpp)

target_link_libraries(IOAccess FRIClient)
