project(SpasticMuscleModel)

cmake_minimum_required(VERSION 3.2)

# Settings.
# ---------
set(TARGET SpasticMuscleModel CACHE TYPE STRING)
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")

# OpenSim uses C++11 language features.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp)

add_library(${TARGET} MODULE ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})

target_compile_definitions(${TARGET} PRIVATE OSIMPLUGIN_EXPORTS)

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
file(GLOB DATA_FILES *.vtp *.obj *.osim)
foreach(dataFile ${DATA_FILES})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR})
endforeach(dataFile)
