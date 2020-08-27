add_library(protobuf_gen_iface INTERFACE)
target_include_directories(protobuf_gen_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(protobuf_gen_iface INTERFACE nanopb_iface)

add_library(protobuf_gen OBJECT FRIMessages.pb.h
                                FRIMessages.pb.c)

target_include_directories(protobuf_gen PUBLIC $<TARGET_PROPERTY:protobuf_gen_iface,INTERFACE_INCLUDE_DIRECTORIES>)
set_target_properties(protobuf_gen PROPERTIES COMPILE_DEFINITIONS "$<TARGET_PROPERTY:protobuf_gen_iface,INTERFACE_COMPILE_DEFINITIONS>")
