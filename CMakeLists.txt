
# Just a guess, can probably go older.
cmake_minimum_required (VERSION 2.8)

project (OutherSpace)

set (CSources
    affine.c
    bitstring.c
    dynamic-setup.c
    kdtree.c
    kptree.c
    material.c
    order.c
    pnm-image.c
    raytrace.c
    scene.c
    simplex.c
    slist.c
    space.c
    testcase.c
    track.c
    util.c
    wavefront-file.c
    xfrm.c
    )

set (GuiName gui)
list (APPEND GuiDFLAGS "NDimensions=3")
list (APPEND GuiDFLAGS "SupportImage")
list (APPEND GuiDFLAGS "SupportOpenGL")
#list (APPEND GuiDFLAGS "INCLUDE_SOURCE")

if (UNIX)
  if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE DEBUG)
  else ()
  endif ()
  set (CMAKE_C_FLAGS "-Wall -Wextra")
  set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ansi -pedantic")
else ()
endif ()

find_package (SDL REQUIRED)
find_package (SDL_image REQUIRED)
find_package (OpenGL REQUIRED)

if (UNIX)
    #find_package (LibSDL REQUIRED)
else ()
  #target_link_libraries (plugin ../SDL-1.2.14/lib/SDL.lib)
endif ()

# Usage:
#   setup_dbgn_defines (${my_sources})
macro (setup_dbgn_defines)
  foreach (src_file ${ARGN})
    set_property(SOURCE ${src_file}
      APPEND PROPERTY COMPILE_DEFINITIONS DBGN_FILENAME="${src_file}")
  endforeach ()
endmacro ()

setup_dbgn_defines (${CSources})
add_executable (gui gui.c motion.c ${CSources})

set_target_properties (gui PROPERTIES
    OUTPUT_NAME "${GuiName}"
  PREFIX ""
  COMPILE_DEFINITIONS "${GuiDFLAGS}"
  LINK_FLAGS "${GuiLFLAGS}")

include_directories (${SDL_INCLUDE_DIR})
include_directories (${SDLMIAGE_INCLUDE_DIR})
include_directories (${OPENGL_INCLUDE_DIR})

target_link_libraries (gui ${SDL_LIBRARY})
target_link_libraries (gui ${SDLIMAGE_LIBRARY})
target_link_libraries (gui ${OPENGL_LIBRARIES})
target_link_libraries (gui m)

