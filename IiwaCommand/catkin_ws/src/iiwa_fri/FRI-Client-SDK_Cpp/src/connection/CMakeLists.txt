add_library(connection OBJECT friUdpConnection.cpp)

if(UNIX)
    target_compile_definitions(connection PRIVATE HAVE_SOCKLEN_T)
endif()
