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

option(DOVRPLOG "Option description" ON)
option(DOSTATS "Option description" ON)
option(OSRMCLIENT "Option description" ON)
option(VRPMINTRACE "Option description" ON)

if(DOVRPLOG)
  add_definitions(-DDOVRPLOG)
endif()

if(DOSTATS)
  add_definitions(-DDOSTATS)
endif()

if(OSRMCLIENT)
  add_definitions(-DOSRMCLIENT)
endif()

if(VRPMINTRACE)
  add_definitions(-DVRPMINTRACE)
endif()

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

## OSRM
# Office
#set(OSRM_SOURCE_PATH "/disco_local/fpacheco/im/osrm-backend")
# Home
set(OSRM_SOURCE_PATH "/home/fpacheco/workspace/idm/right-side/osrm-backend")
set(OSRM_LIBRARIES ${OSRM_SOURCE_PATH}/build/libOSRM.a)
##


add_library (${VRPTOOLS_LIBRARY_NAME} SHARED ${VRPTOOLS_SOURCES})
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CPP)
set_target_properties(${VRPTOOLS_LIBRARY_NAME} PROPERTIES VERSION ${VRPTOOLS_VERSION_STRING})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Threads_LIBRARIES} ${OSRM_LIBRARIES})

add_subdirectory (tests)
