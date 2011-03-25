cmake_minimum_required(VERSION 2.8)
project(flash)

# find pkg-config
find_package(PkgConfig REQUIRED)

# check for Player/Stage C++ libraries
pkg_check_modules(PLAYERCPP REQUIRED playerc++)

# includes
include_directories(${PLAYERCPP_INCLUDE_DIRS})

add_library(flash SHARED flash.cpp flashsip.cpp packet.cpp robot_params.cpp kinecalc.cpp)

target_link_libraries(flash ${PLAYERCPP_LIBRARIES})
