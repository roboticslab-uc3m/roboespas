add_library(client_lbr_iface INTERFACE)
target_link_libraries(client_lbr_iface INTERFACE base_iface protobuf_iface)

add_library(client_lbr OBJECT friLBRClient.cpp
                              friLBRCommand.cpp
                              friLBRState.cpp)

target_include_directories(client_lbr PRIVATE $<TARGET_PROPERTY:client_lbr_iface,INTERFACE_INCLUDE_DIRECTORIES>)
set_target_properties(client_lbr PROPERTIES COMPILE_DEFINITIONS "$<TARGET_PROPERTY:client_lbr_iface,INTERFACE_COMPILE_DEFINITIONS>")
