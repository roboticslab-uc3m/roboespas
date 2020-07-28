add_library(nanopb_iface INTERFACE)
target_include_directories(nanopb_iface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(nanopb_iface INTERFACE PB_FIELD_16BIT)

if(WIN32 AND NOT CYGWIN)
    target_compile_definitions(nanopb_iface INTERFACE PB_SYSTEM_HEADER=\"pb_syshdr_win.h\")
else()
    target_compile_definitions(nanopb_iface INTERFACE PB_SYSTEM_HEADER=\"pb_syshdr.h\"
                                                      HAVE_STDINT_H
                                                      HAVE_STDDEF_H
                                                      HAVE_STDBOOL_H
                                                      HAVE_STDLIB_H
                                                      HAVE_STRING_H)
endif()

add_library(nanopb OBJECT pb.h
                          pb_syshdr.h
                          pb_syshdr_win.h
                          pb_decode.h
                          pb_decode.c
                          pb_encode.h
                          pb_encode.c)

set_target_properties(nanopb PROPERTIES COMPILE_DEFINITIONS "$<TARGET_PROPERTY:nanopb_iface,INTERFACE_COMPILE_DEFINITIONS>")
