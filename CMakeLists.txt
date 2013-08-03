
# To see all commands:
#   make VERBOSE=1

# To use on windows, do:
#   cmake -G "NMake Makefiles"
#   nmake

# Just a guess, can probably go older.
cmake_minimum_required (VERSION 2.8)

project (OutherSpace)

set (CxPath ../cx)
set (BinPath ../bin)
set (BldPath outherspace)

set (CommonCFiles
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
    )

list (APPEND CFiles
    ${CommonCFiles}
    cli.c
    gui.c
    imgdiff.c
    motion.c
    file2array.c
    verif/main.c
    verif/order.c
    verif/pack.c
    )

list (APPEND HFiles
    affine.h
    bbox.h
    bitstring.h
    color.h
    dynamic-setup.h
    gui-indep.c
    gui-indep.h
    gui-opengl.c
    kdtree.h
    kptree.h
    lightcut.h
    material.h
    motion.h
    op.h
    order.h
    point.h
    point2.h
    pnm-image.h
    raytrace.h
    scene.h
    serial.h
    space.h
    space-junk.h
    simplex.h
    slist.h
    testcase.h
    track.h
    util.h
    wavefront-file.h
    xfrm.h
    )

include (${CxPath}/include.cmake)

#set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BinPath})

file (MAKE_DIRECTORY ${BldPath}/verif)

foreach (f ${CommonCFiles})
    list (APPEND FullCommonCFiles ${BldPath}/${f})
endforeach ()

add_executable (cli
    ${BldPath}/cli.c
    ${BldPath}/util.c
    ${FullCommonCFiles}
    )
set_target_properties (cli PROPERTIES
    OUTPUT_NAME cli)
target_link_libraries (cli CxLib)

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
    #set (CMAKE_C_FLAGS "-Wall -Wextra -ansi -pedantic")
    find_package (SDL REQUIRED)
    find_package (SDL_image REQUIRED)
    find_package (OpenMP REQUIRED)
else ()
    set (CMAKE_C_FLAGS "/W4 /MP")
    # Disable warning: 'fopen' unsafe, use fopen_s instead
    add_definitions ("-D_CRT_SECURE_NO_WARNINGS")
    # Other warnings disabled in util.h via #pragmas.

    set (CMAKE_BUILD_TYPE RELEASE)
    set (SDL_pfx "../../SDL-1.2.14")
    set (SDL_LIBRARY "${SDL_pfx}/lib/SDL")
    list (APPEND SDL_LIBRARY "${SDL_pfx}/lib/SDLmain")
    set (SDL_INCLUDE_DIR "${SDL_pfx}/include")

    set (SDL_image_pfx "../../SDL_image-1.2.10")
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

add_executable (imgdiff
    ${BldPath}/imgdiff.c
    ${BldPath}/pnm-image.c
    ${BldPath}/util.c
    )
set_target_properties (imgdiff PROPERTIES
    OUTPUT_NAME imgdiff)
target_link_libraries (imgdiff CxLib)


add_executable (file2array
    ${BldPath}/file2array.c
    ${BldPath}/util.c
    )
set_target_properties (file2array PROPERTIES
    OUTPUT_NAME file2array)
target_link_libraries (file2array CxLib)

set (EmbedSfx ".embed.h")

add_custom_command (OUTPUT ${BldPath}/"phong.glsl${EmbedSfx}"
    COMMAND file2array ${BldPath}/"phong.glsl${EmbedSfx}" phong.vert phong.frag 4d.vert
    DEPENDS phong.vert phong.frag 4d.vert)

add_custom_command (OUTPUT ${BldPath}/"perturb.cl${EmbedSfx}"
    COMMAND file2array ${BldPath}/"perturb.cl${EmbedSfx}" perturb.cl
    DEPENDS perturb.cl)

add_executable (verify
    ${BldPath}/verif/main.c
    ${BldPath}/verif/order.c
    ${BldPath}/verif/pack.c
    ${FullCommonCFiles}
    )
set_target_properties (verify PROPERTIES
    OUTPUT_NAME verify)
target_link_libraries (verify CxLib)

add_executable (gui
    ${BldPath}/gui.c
    ${BldPath}/motion.c
    ${BldPath}/"phong.glsl${EmbedSfx}"
    ${BldPath}/"perturb.cl${EmbedSfx}"
    ${FullCommonCFiles}
    )
set_target_properties (gui PROPERTIES
    OUTPUT_NAME "${GuiName}"
    COMPILE_DEFINITIONS "${GuiDFLAGS}"
    LINK_FLAGS "${GuiLFLAGS}")
target_link_libraries (gui CxLib)
#add_executable (gui gui.c motion.c
#    "phong.glsl${EmbedSfx}"
#    "perturb.cl${EmbedSfx}"
#    ${CSources})
#set_target_properties (gui PROPERTIES
#    OUTPUT_NAME "${GuiName}"
#    COMPILE_DEFINITIONS "${GuiDFLAGS}"
#    LINK_FLAGS "${GuiLFLAGS}")
#
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif ()

#include_directories (".")
include_directories (${BldPath})
include_directories (SYSTEM ${SDL_INCLUDE_DIR})
include_directories (SYSTEM ${SDLIMAGE_INCLUDE_DIR})
include_directories (SYSTEM ${OPENGL_INCLUDE_DIR})

target_link_libraries (gui ${SDL_LIBRARY})
target_link_libraries (gui ${SDLIMAGE_LIBRARY})
target_link_libraries (gui ${OPENGL_LIBRARIES})

if (UNIX)
    target_link_libraries (imgdiff m)
    target_link_libraries (file2array m)
    target_link_libraries (gui m)
    target_link_libraries (cli m)
    target_link_libraries (verify m)
endif ()

