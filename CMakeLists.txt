cmake_minimum_required(VERSION 3.1)

project(mc-hoap3)

set(CMAKE_CXX_STANDARD 11)

find_package(hoap3_description REQUIRED)
find_package(mc_rtc REQUIRED)

find_description_package(hoap3_description)
message(STATUS "HOAP3_DESCRIPTION_PATH: ${HOAP3_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(hoap3 src/hoap3.cpp src/hoap3.h)
target_include_directories(hoap3 PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()

install(FILES etc/LIPMWalking/hoap3.yaml
        DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/LIPMWalking/)
