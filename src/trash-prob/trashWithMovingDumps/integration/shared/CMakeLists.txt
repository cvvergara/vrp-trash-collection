cmake_minimum_required (VERSION 2.8.8)

project(vrptools)
message ("Generating project ${PROJECT_NAME}")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
	add_definitions(-D_FORTIFY_SOURCE=2)
endif()

set(VRPTOOLS_LIBRARY_NAME "vrptools")
set(DEPS_LIBRARY_NAME "deps")

set(VRPTOOLS_VERSION_MAJOR 0)
set(VRPTOOLS_VERSION_MINOR 1)
set(VRPTOOLS_VERSION_PATCH 0)

include_directories(
    "../../../initTrash"
    "../../../../baseClasses"
    "../../solverTrash"
    "../../../baseTrash"
)

#set(CMAKE_CPP_FLAGS "-Wall -ansi -pedantic -Wno-long-long -Wconversion -Wformat=2 -Wshadow -Wcast-qual -Wwrite-strings -fPIC")
#set(CMAKE_CPP_FLAGS_DEBUG "-g")
#set(CMAKE_CPP_FLAGS_RELEASE "-O2")

# Append the source files here
list (APPEND VRPTOOLS_SOURCES
  vrptools.cpp
)

# Append the header files here. this will get copied to include directory
list (APPEND VRPTOOLS_INCLUDE_FILES
  vrptools.h
)

add_library (core OBJECT ${VRPTOOLS_SOURCES})
add_library (${VRPTOOLS_LIBRARY_NAME} SHARED $<TARGET_OBJECTS:core>)

