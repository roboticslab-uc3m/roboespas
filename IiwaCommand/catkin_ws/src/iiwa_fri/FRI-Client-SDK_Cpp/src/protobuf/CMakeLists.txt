add_library(protobuf_iface INTERFACE)
target_include_directories(protobuf_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(protobuf_iface INTERFACE nanopb_iface protobuf_gen_iface)

add_library(protobuf OBJECT friCommandMessageEncoder.h
                            friCommandMessageEncoder.cpp
                            friMonitoringMessageDecoder.h
                            friMonitoringMessageDecoder.cpp
                            pb_frimessages_callbacks.h
                            pb_frimessages_callbacks.c)

target_include_directories(protobuf PUBLIC $<TARGET_PROPERTY:protobuf_iface,INTERFACE_INCLUDE_DIRECTORIES>)
set_target_properties(protobuf PROPERTIES COMPILE_DEFINITIONS "$<TARGET_PROPERTY:protobuf_iface,INTERFACE_COMPILE_DEFINITIONS>")
