
cmake_minimum_required (VERSION 2.8)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

include(ExternalProject)

set (TopPath ${CMAKE_CURRENT_SOURCE_DIR})
set (DepPath ${TopPath}/dep)
set (BinPath ${TopPath}/bin)

ExternalProject_Add(cx
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/external/cx
  SOURCE_DIR ${DepPath}/cx/src
  BINARY_DIR ${DepPath}/cx/bld
  INSTALL_COMMAND echo "No install step."
  )

ExternalProject_Add(outherspace
  DEPENDS cx
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/external/src
  SOURCE_DIR ${TopPath}/src
  BINARY_DIR ${TopPath}/bld
  CONFIGURE_COMMAND ${CMAKE_COMMAND}
  ../src
  INSTALL_COMMAND echo "No install step."
  )

