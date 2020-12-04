cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Start a project.
project(FRI-Client-SDK LANGUAGES C CXX)

# Pick up local CMake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Pick headers.
include_directories(${CMAKE_SOURCE_DIR}/include)

# Traverse subdirectories.
add_subdirectory(src)
add_subdirectory(example)

# Create library target.
add_library(FRIClient STATIC $<TARGET_OBJECTS:base>
                             $<TARGET_OBJECTS:client_lbr>
                             $<TARGET_OBJECTS:client_trafo>
                             $<TARGET_OBJECTS:connection>
                             $<TARGET_OBJECTS:nanopb>
                             $<TARGET_OBJECTS:protobuf>
                             $<TARGET_OBJECTS:protobuf_gen>)

# Register include directories.
target_include_directories(FRIClient PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install library.
install(TARGETS FRIClient
        EXPORT FRIClient
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Install headers.
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX})

# Store project in the user registry.
export(PACKAGE FRIClient)

# Register build-tree export target.
export(EXPORT FRIClient
       NAMESPACE ROBOTICSLAB::
       FILE FRIClientConfig.cmake)

# Pick installation path for CMake files.
if(WIN32 AND NOT CYGWIN)
    set(_cmake_install_path CMake)
else()
    set(_cmake_install_path ${CMAKE_INSTALL_LIBDIR}/cmake/FRIClient)
endif()

# Register install-tree export target.
install(EXPORT FRIClient
        DESTINATION ${_cmake_install_path}
        NAMESPACE ROBOTICSLAB::
        FILE FRIClientConfig.cmake)

# Clear local variable.
unset(_cmake_install_path)

# Configure and create uninstall target.
include(AddUninstallTarget)
