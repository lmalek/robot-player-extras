cmake_minimum_required(VERSION 2.8)
project(te)

# find pkg-config
find_package(PkgConfig REQUIRED)

# check for Player/Stage C++ libraries
pkg_check_modules(PLAYERCPP REQUIRED playerc++)

# includes
include_directories(${PLAYERCPP_INCLUDE_DIRS})

add_library(te SHARED te.cpp)
