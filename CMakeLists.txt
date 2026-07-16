cmake_minimum_required(VERSION 3.18)
project(scrcpy-rfb C)

include(FetchContent)
find_package(Threads REQUIRED)

# Pinned upstream LibVNC/libvncserver revision; patches/ applies on top.
set(LIBVNCSERVER_COMMIT "42494999e6492aaab9c1db785ecd293ef10b3aed"
    CACHE STRING "Upstream LibVNC/libvncserver commit to build against")
set(FFMPEG_ROOT "" CACHE PATH "Prefix of an FFmpeg installation (avcodec/avutil/swscale)")

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(WITH_ZLIB ON CACHE BOOL "" FORCE)
set(WITH_JPEG ON CACHE BOOL "" FORCE)
set(WITH_LZO OFF CACHE BOOL "" FORCE)
set(WITH_PNG OFF CACHE BOOL "" FORCE)
set(WITH_SDL OFF CACHE BOOL "" FORCE)
set(WITH_GTK OFF CACHE BOOL "" FORCE)
set(WITH_LIBSSH2 OFF CACHE BOOL "" FORCE)
set(WITH_GNUTLS OFF CACHE BOOL "" FORCE)
set(WITH_OPENSSL OFF CACHE BOOL "" FORCE)
set(WITH_SYSTEMD OFF CACHE BOOL "" FORCE)
set(WITH_GCRYPT OFF CACHE BOOL "" FORCE)
set(WITH_FFMPEG OFF CACHE BOOL "" FORCE)
set(WITH_WEBSOCKETS OFF CACHE BOOL "" FORCE)
set(WITH_SASL OFF CACHE BOOL "" FORCE)
set(WITH_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WITH_TESTS OFF CACHE BOOL "" FORCE)
set(WITH_LIBVNCCLIENT OFF CACHE BOOL "" FORCE)
set(LIBVNCSERVER_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(libvncserver
  URL https://github.com/LibVNC/libvncserver/archive/${LIBVNCSERVER_COMMIT}.tar.gz
  PATCH_COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/patches/apply.cmake
)
FetchContent_MakeAvailable(libvncserver)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h HINTS ${FFMPEG_ROOT}/include)
find_library(AVCODEC_LIBRARY avcodec HINTS ${FFMPEG_ROOT}/lib)
find_library(AVUTIL_LIBRARY avutil HINTS ${FFMPEG_ROOT}/lib)
find_library(SWSCALE_LIBRARY swscale HINTS ${FFMPEG_ROOT}/lib)
if(NOT AVCODEC_INCLUDE_DIR OR NOT AVCODEC_LIBRARY OR NOT AVUTIL_LIBRARY OR NOT SWSCALE_LIBRARY)
  message(FATAL_ERROR "FFmpeg avcodec/avutil/swscale not found; set FFMPEG_ROOT")
endif()

add_executable(scrcpy-rfb src/scrcpy-rfb.c)
target_compile_definitions(scrcpy-rfb PRIVATE _GNU_SOURCE)
target_compile_options(scrcpy-rfb PRIVATE -Wall -Wextra -Werror)
target_include_directories(scrcpy-rfb PRIVATE
  ${AVCODEC_INCLUDE_DIR}
  ${libvncserver_SOURCE_DIR}/include
  ${libvncserver_BINARY_DIR}/include
)
target_link_libraries(scrcpy-rfb PRIVATE
  vncserver
  ${SWSCALE_LIBRARY}
  ${AVCODEC_LIBRARY}
  ${AVUTIL_LIBRARY}
  m
  Threads::Threads
)
