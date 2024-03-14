cmake_minimum_required(VERSION 3.1)

project(mc-hoap3)

set(CMAKE_CXX_STANDARD 11)

find_package(hoap3_description REQUIRED)
find_package(mc_rtc REQUIRED)

if("${hoap3_description_INSTALL_PREFIX}" STREQUAL "")
  if("${hoap3_description_SOURCE_PREFIX}" STREQUAL "")
    message(
      FATAL_ERROR
        "Your hoap3_description package does not define where to find the data")
  endif()
  set(HOAP3_DESCRIPTION_PATH "${hoap3_description_SOURCE_PREFIX}")
else()
  if(EXISTS "${hoap3_description_INSTALL_PREFIX}/share/hoap3_description")
    set(HOAP3_DESCRIPTION_PATH
        "${hoap3_description_INSTALL_PREFIX}/share/hoap3_description")
  else()
    set(HOAP3_DESCRIPTION_PATH "${hoap3_description_INSTALL_PREFIX}")
  endif()
endif()

message("HOAP3_DESCRIPTION_PATH: ${HOAP3_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(hoap3 src/hoap3.cpp src/hoap3.h)
target_include_directories(hoap3 PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
