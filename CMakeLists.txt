
# To see all commands:
#   make VERBOSE=1

# To use on windows, do:
#   cmake -G "NMake Makefiles"
#   nmake

# Just a guess, can probably go older.
cmake_minimum_required (VERSION 2.8)

project (OutherSpace)

set (bin_path ../bin)
set (cx_path ../cx)

set (CSources
    affine.c
    bbox.c
    bitstring.c
    dynamic-setup.c
    kdtree.c
    kptree.c
    lightcut.c
    material.c
    order.c
    pnm-image.c
    raytrace.c
    scene.c
    serial.c
    simplex.c
    slist.c
    testcase.c
    track.c
    util.c
    wavefront-file.c
    xfrm.c
    ${cx_path}/bstree.c
    ${cx_path}/fileb.c
    ${cx_path}/sys-cx.c
    )

set (GuiName gui)
list (APPEND GuiDFLAGS "NDimensions=3")
list (APPEND GuiDFLAGS "SupportImage")
list (APPEND GuiDFLAGS "SupportOpenGL")
list (APPEND GuiDFLAGS "EmbedFiles")
#list (APPEND GuiDFLAGS "INCLUDE_SOURCE")

if (UNIX)
    if (NOT CMAKE_BUILD_TYPE)
        set (CMAKE_BUILD_TYPE DEBUG)
    else ()
    endif ()
    set (CMAKE_C_FLAGS "-Wall -Wextra -ansi -pedantic")
    find_package (SDL REQUIRED)
    find_package (SDL_image REQUIRED)
    find_package (OpenMP REQUIRED)
else ()
    set (CMAKE_C_FLAGS "/W4 /MP")
    # Disable warning: 'fopen' unsafe, use fopen_s instead
    add_definitions ("-D_CRT_SECURE_NO_WARNINGS")
    # Other warnings disabled in util.h via #pragmas.

    set (CMAKE_BUILD_TYPE RELEASE)
    set (SDL_pfx "../SDL-1.2.14")
    set (SDL_LIBRARY "${SDL_pfx}/lib/SDL")
    list (APPEND SDL_LIBRARY "${SDL_pfx}/lib/SDLmain")
    set (SDL_INCLUDE_DIR "${SDL_pfx}/include")

    set (SDL_image_pfx "../SDL_image-1.2.10")
    set (SDLIMAGE_INCLUDE_DIR "${SDL_image_pfx}/include")
    set (SDLIMAGE_LIBRARY "${SDL_image_pfx}/lib/SDL_image")
endif ()

find_package (OpenGL REQUIRED)

# Usage:
#   setup_dbgn_defines (${my_sources})
macro (setup_dbgn_defines)
    foreach (src_file ${ARGN})
        set_property(SOURCE ${src_file}
            APPEND PROPERTY COMPILE_DEFINITIONS DBGN_FILENAME="${src_file}")
    endforeach ()
endmacro ()

setup_dbgn_defines (${CSources})

add_executable (file2array file2array.c util.c ${cx_path}/fileb.c)
set_target_properties (file2array PROPERTIES
    PREFIX "${bin_path}/" OUTPUT_NAME file2array)


set (EmbedSfx ".embed.h")

add_custom_command (OUTPUT "phong.glsl${EmbedSfx}"
    COMMAND file2array "phong.glsl${EmbedSfx}" phong.vert phong.frag 4d.vert
    DEPENDS phong.vert phong.frag 4d.vert)

add_custom_command (OUTPUT "perturb.cl${EmbedSfx}"
    COMMAND file2array "perturb.cl${EmbedSfx}" perturb.cl
    DEPENDS perturb.cl)

add_executable (verify
    verif/main.c
    verif/order.c
    verif/pack.c
    ${CSources})
set_target_properties (verify PROPERTIES
    PREFIX "${bin_path}/" OUTPUT_NAME verify)

add_executable (gui gui.c motion.c
    "phong.glsl${EmbedSfx}"
    "perturb.cl${EmbedSfx}"
    ${CSources})
set_target_properties (gui PROPERTIES
    PREFIX "${bin_path}/" OUTPUT_NAME "${GuiName}"
    COMPILE_DEFINITIONS "${GuiDFLAGS}"
    LINK_FLAGS "${GuiLFLAGS}")

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif ()

include_directories (".")
include_directories ("..")
include_directories (SYSTEM ${SDL_INCLUDE_DIR})
include_directories (SYSTEM ${SDLIMAGE_INCLUDE_DIR})
include_directories (SYSTEM ${OPENGL_INCLUDE_DIR})

target_link_libraries (gui ${SDL_LIBRARY})
target_link_libraries (gui ${SDLIMAGE_LIBRARY})
target_link_libraries (gui ${OPENGL_LIBRARIES})

if (UNIX)
    target_link_libraries (file2array m)
    target_link_libraries (gui m)
    target_link_libraries (verify m)
endif ()

