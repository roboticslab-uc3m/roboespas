add_library(base_iface INTERFACE)
target_include_directories(base_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(base_iface INTERFACE protobuf_iface protobuf_gen_iface)

add_library(base OBJECT friClientData.h
                        friClientApplication.cpp)

target_include_directories(base PUBLIC $<TARGET_PROPERTY:base_iface,INTERFACE_INCLUDE_DIRECTORIES>)
set_target_properties(base PROPERTIES COMPILE_DEFINITIONS "$<TARGET_PROPERTY:base_iface,INTERFACE_COMPILE_DEFINITIONS>")
