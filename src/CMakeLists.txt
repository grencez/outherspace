
# To see all commands:
#   make VERBOSE=1

# To use on windows, do:
#   cmake -G "NMake Makefiles"
#   nmake

# Just a guess, can probably go older.
cmake_minimum_required (VERSION 2.8)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project (OutherSpace)

set (TopPath ${CMAKE_CURRENT_SOURCE_DIR}/..)
set (BinPath ${TopPath}/bin)
set (CxTopPath ${TopPath}/dep/cx)
list (APPEND CMAKE_MODULE_PATH ${CxTopPath}/cmake)
set (BldPath outherspace)

set (CommonCFiles
    affine.c
    bbox.c
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
    verif/main.c
    verif/order.c
    verif/pack.c
    )

list (APPEND HFiles
    affine.h
    bbox.h
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

list (APPEND CFiles hello.c)

include (${CxTopPath}/src/include.cmake)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BinPath})
#set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

file (MAKE_DIRECTORY ${BldPath}/verif)

find_library (M_LIB m)
find_package (SDL)
find_package (SDL_image)
find_package (OpenMP)
find_package (OpenGL)
find_package (OpenCL)

addbinexe (cli
  cli.c
  util.c
  ${CommonCFiles}
  )
target_link_libraries (cli CxLib ${M_LIB})


list (APPEND GuiDFLAGS "EmbedFiles")

if (NOT UNIX)
  if (NOT SDL_FOUND)
    set (SDL_pfx "../../SDL-1.2.14")
    set (SDL_LIBRARY "${SDL_pfx}/lib/SDL")
    list (APPEND SDL_LIBRARY "${SDL_pfx}/lib/SDLmain")
    set (SDL_INCLUDE_DIR "${SDL_pfx}/include")
    set (SDL_FOUND YES)
  endif ()

  if (NOT SDL_IMAGE_FOUND)
    set (SDL_image_pfx "../../SDL_image-1.2.10")
    set (SDL_IMAGE_INCLUDE_DIR "${SDL_image_pfx}/include")
    set (SDL_IMAGE_LIBRARY "${SDL_image_pfx}/lib/SDL_image")
    set (SDL_IMAGE_FOUND YES)
  endif ()
endif ()

if (SDL_IMAGE_FOUND)
  list (APPEND GuiDFLAGS "SupportImage")
endif ()

if (OPENGL_FOUND)
  list (APPEND GuiDFLAGS "SupportOpenGL")
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

addbinexe (imgdiff
  imgdiff.c
  pnm-image.c
  util.c
  )
target_link_libraries (imgdiff CxLib ${M_LIB})


set (EmbedSfx .embed.h)

add_custom_command (OUTPUT ${BldPath}/phong.glsl${EmbedSfx}
  COMMAND cembed -o ${BldPath}/phong.glsl${EmbedSfx} phong.vert phong.frag 4d.vert
  DEPENDS phong.vert phong.frag 4d.vert
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_command (OUTPUT ${BldPath}/perturb.cl${EmbedSfx}
  COMMAND cembed -o ${BldPath}/perturb.cl${EmbedSfx} perturb.cl
  DEPENDS perturb.cl
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target (EmbedSources SOURCES ${BldPath}/phong.glsl${EmbedSfx} ${BldPath}/perturb.cl${EmbedSfx})

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (SDL_IMAGE_FOUND)
  include_directories (SYSTEM ${SDL_IMAGE_INCLUDE_DIR})
endif ()

#### Test Executable ####
addbinexe (verify
  verif/main.c
  verif/order.c
  verif/pack.c
  ${CommonCFiles}
  )
set_target_properties (verify PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_property (TARGET verify
  APPEND PROPERTY INCLUDE_DIRECTORIES ${BldPath})
target_link_libraries (verify CxLib ${M_LIB})

add_custom_command (
  OUTPUT ${BldPath}/verif/testswitch.c
  COMMAND cswitch testname
  -case-pfx "fn = testfn_"
  -array AllTests
  -x ${CMAKE_CURRENT_SOURCE_DIR}/verif/testlist.txt
  -o ${BldPath}/verif/testswitch.c
  DEPENDS cswitch ${CMAKE_CURRENT_SOURCE_DIR}/verif/testlist.txt)
add_custom_target (TestSwitchSources SOURCES ${BldPath}/verif/testswitch.c)
add_dependencies(verify TestSwitchSources)

#### GUI Executables ####
function (add_gui_exe tgt oname)
  addbinexe (${tgt}
    gui.c
    motion.c
    ${CommonCFiles}
    )

  add_dependencies(${tgt} EmbedSources)

  set_property (TARGET ${tgt}
    APPEND PROPERTY INCLUDE_DIRECTORIES ${SDL_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})

  set (tmpDFLAGS ${GuiDFLAGS})
  foreach (dflag ${ARGN})
    list (APPEND tmpDFLAGS ${dflag})
  endforeach ()

  set_target_properties (${tgt} PROPERTIES
    OUTPUT_NAME "${oname}"
    COMPILE_DEFINITIONS "${tmpDFLAGS}"
    LINK_FLAGS "${GuiLFLAGS}")

  target_link_libraries (${tgt} CxLib)
  target_link_libraries (${tgt} ${SDL_LIBRARY})
  if (SDL_IMAGE_FOUND)
    target_link_libraries (${tgt} ${SDL_IMAGE_LIBRARY})
  endif ()
  target_link_libraries (${tgt} ${OPENGL_LIBRARIES} ${M_LIB})
endfunction ()

if (SDL_FOUND AND OPENGL_FOUND)
  add_gui_exe (gui gui "NDimensions=3")
  add_gui_exe (outherspace outherspace "NDimensions=3" "UseRaceCraft")
  add_gui_exe (outherspace4d outherspace4d "NDimensions=4" "UseRaceCraft")
endif ()

if (OPENCL_FOUND)
  addbinexe (hello hello.c ${CommonCFiles})
  set_target_properties (hello PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_property (TARGET hello
    APPEND PROPERTY INCLUDE_DIRECTORIES ${OPENCL_INCLUDE_DIR})
  target_link_libraries (hello ${OPENCL_LIBRARIES} ${M_LIB})
endif ()


enable_testing ()

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/verif/testlist.txt TestNames)
string(REPLACE "\n" ";" TestNames "${TestNames}")

foreach (testname ${TestNames})
  add_test (NAME ${testname}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND verify ${testname})
endforeach ()


