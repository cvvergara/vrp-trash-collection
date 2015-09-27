cmake_minimum_required (VERSION 2.8.8)

project(vrptools)
message ("Generating project ${PROJECT_NAME}")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR AND NOT MSVC_IDE)
  MESSAGE(FATAL_ERROR "In-source builds are not allowed.
Please create a directory and run cmake from there, passing the path
to this source directory as the last argument.
This process created the file `CMakeCache.txt' and the directory
`CMakeFiles'. Please delete them.")
endif()

set(VRPTOOLS_LIBRARY_NAME "vrptools")
set(DEPS_LIBRARY_NAME "deps")

set(VRPTOOLS_VERSION_MAJOR 0)
set(VRPTOOLS_VERSION_MINOR 1)
set(VRPTOOLS_VERSION_PATCH 0)
set(VRPTOOLS_VERSION_STRING ${VRPTOOLS_VERSION_MAJOR}.${VRPTOOLS_VERSION_MINOR}.${VRPTOOLS_VERSION_PATCH})


set(DOVRPLOG 1)
set(DOSTATS 1)

include_directories(
    "../../../initTrash"
    "../../../../baseClasses"
    "../../solverTrash"
    "../../../baseTrash"
)

# Activa c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CPP_FLAGS "-Wall -fPIC")
set(CMAKE_CPP_FLAGS_DEBUG "-g")
set(CMAKE_CPP_FLAGS_RELEASE "-O2")

# Append the source files here
set (VRPTOOLS_SOURCES
  vrptools.cpp
)

# Append the header files here. this will get copied to include directory
set (VRPTOOLS_INCLUDE_FILES
  vrptools.h
)

### Find
## Boost
set(BOOST_COMPONENTS date_time filesystem iostreams program_options
regex system thread unit_test_framework)
#Check Boost
set(BOOST_MIN_VERSION "1.49.0")
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Fatal error: Boost (version >= 1.49.0) required.\n")
endif()
##

add_library (${VRPTOOLS_LIBRARY_NAME} SHARED ${VRPTOOLS_SOURCES})
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CPP)
set_target_properties(${VRPTOOLS_LIBRARY_NAME} PROPERTIES VERSION ${VRPTOOLS_VERSION_STRING})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Threads_LIBRARIES} ${OSRM_LIBRARIES})
