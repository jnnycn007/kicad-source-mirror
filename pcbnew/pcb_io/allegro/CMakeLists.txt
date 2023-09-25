set( ALLEGRO2PCBNEW_SRCS
    allegro_file.cpp
    allegro_parser.cpp
    allegro_pcb.cpp
    pcb_io_allegro.cpp
    )

add_library( allegro2pcbnew STATIC ${ALLEGRO2PCBNEW_SRCS} )

target_link_libraries (allegro2pcbnew
    pcbcommon
    )

target_include_directories( allegro2pcbnew
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
