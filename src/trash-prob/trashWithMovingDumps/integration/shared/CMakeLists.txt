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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(FindPkgConfig)

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
option(VRPMAXTRACE "Option description" OFF)

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

if(OSRMCLIENTTRACE)
  add_definitions(-DOSRMCLIENTTRACE)
endif()

if(VRPMAXTRACE)
  add_definitions(-DVRPMAXTRACE)
endif()

include_directories(
    "../../../initTrash"
    "../../../../baseClasses"
    "../../solverTrash"
    "../../../baseTrash"
)

# Activa c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CPP_FLAGS "-std=c++11 -Wall -fPIC")
set(CMAKE_CPP_FLAGS_DEBUG "-std=c++11 -g")
set(CMAKE_CPP_FLAGS_RELEASE "-std=c++11 -O2")

### Find
## Boost
set(BOOST_COMPONENTS date_time filesystem iostreams program_options regex system thread unit_test_framework)
#Check Boost
set(BOOST_MIN_VERSION "1.49.0")
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Fatal error: Boost (version >= 1.49.0) required.\n")
endif()
##

## Threads
find_package(Threads REQUIRED)
##

## OSRM
# Office
set(OSRM_SOURCE_PATH "/disco_local/fpacheco/idm/right-side/osrm-backend")
# Home
#set(OSRM_SOURCE_PATH "/home/fpacheco/workspace/idm/right-side/osrm-backend")
#set(OSRM_INCLUDE_DIRS ${OSRM_SOURCE_PATH}/build/libOSRM.a)
set(OSRM_LIBRARIES ${OSRM_SOURCE_PATH}/build/libOSRM.a)
##

## Glog
find_package(Glog REQUIRED)
##

## CURL
find_package(CURL REQUIRED)
##

## CURLPP via PKG
if (UNIX)
  PKG_CHECK_MODULES(CURLPP curlpp)
  if (CURLPP_FOUND EQUAL 1)
    MESSAGE(STATUS "curlpp found" )
  else()
    MESSAGE(FATAL_ERROR "curlpp library NOT found (required)!")
  endif()
endif(UNIX)
###

## rapidjson
find_package(rapidjson REQUIRED)
##

### Includes
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CURLPP_INCLUDE_DIRS})
include_directories(${RAPIDJSON_INCLUDE_DIRS})
###

# base trash
set (BASECLASSES_SOURCES
    ../../../../baseClasses/basictypes.cpp
    #../../../../baseClasses/config.cpp
    ../../../../baseClasses/md5.cpp
    ../../../../baseClasses/move.cpp
    ../../../../baseClasses/node.cpp
    ../../../../baseClasses/phantomnode.cpp
    ../../../../baseClasses/osrmclient.cpp
    ../../../../baseClasses/signalhandler.cpp
    ../../../../baseClasses/stats.cpp
    ../../../../baseClasses/tweval.cpp
    ../../../../baseClasses/twnode.cpp
    ../../../../baseClasses/vrp_assert.cpp
    ../../../../baseClasses/vrposrm.cpp
)

set (BASECLASSES_INCLUDE_FILES
    ../../../../baseClasses/basictypes.h
    ../../../../baseClasses/logger.h
    ../../../../baseClasses/move.h
    ../../../../baseClasses/osrmclient.h
    ../../../../baseClasses/plot.h
    ../../../../baseClasses/singleton.h
    ../../../../baseClasses/timer.h
    ../../../../baseClasses/twc.h
    ../../../../baseClasses/twnode.h
    ../../../../baseClasses/vrp_assert.h
    #../../../../baseClasses/config.h
    ../../../../baseClasses/md5.h
    ../../../../baseClasses/node.h
    ../../../../baseClasses/phantomnode.h
    ../../../../baseClasses/pg_types_vrp.h
    ../../../../baseClasses/signalhandler.h
    ../../../../baseClasses/stats.h
    ../../../../baseClasses/twbucket.h
    ../../../../baseClasses/tweval.h
    ../../../../baseClasses/twpath.h
    ../../../../baseClasses/vrposrm.h
)

set (BASETRASH_SOURCES
    ../../../baseTrash/basevehicle.cpp
    #../../../baseTrash/extendedvehicle.cpp
    ../../../baseTrash/prob_trash.cpp
    ../../../baseTrash/solution.cpp
    #../../../baseTrash/trashconfig.cpp
    ../../../baseTrash/trashprob.cpp
)

set (BASETRASH_INCLUDE_FILES
    ../../../baseTrash/basevehicle.h
    #../../../baseTrash/extendedvehicle.h
    ../../../baseTrash/prob_trash.h
    ../../../baseTrash/solution.h
    #../../../baseTrash/trashconfig.h
    ../../../baseTrash/trashnode.h
    ../../../baseTrash/trashprob.h
)

set (INITTRASH_SOURCES
    ../../../initTrash/truckManyVisitsDump.cpp
    ../../../initTrash/basicOperations.cpp
    ../../../initTrash/givenInitialSol.cpp
)

set (INITTRASH_INCLUDE_FILES
    ../../../initTrash/truckManyVisitsDump.h
    ../../../initTrash/basicOperations.h
    ../../../initTrash/givenInitialSol.h
)


set (SOLVERTRASH_SOURCES
    ../../solverTrash/costvehicle.cpp
    ../../solverTrash/fleetOpt.cpp
    ../../solverTrash/optsol.cpp
    ../../solverTrash/tabuopt.cpp
    ../../solverTrash/tripVehicle.cpp
    ../../solverTrash/vehicle.cpp
)

set (SOLVERTRASH_INCLUDE_FILES
    ../../solverTrash/costvehicle.h
    ../../solverTrash/fleetOpt.h
    ../../solverTrash/optsol.h
    ../../solverTrash/tabuopt.h
    ../../solverTrash/tripVehicle.h
    ../../solverTrash/vehicle.h
)


# Append the source files here
set (VRPTOOLS_SOURCES
  loadfromfiles.cpp
  vrptools.cpp
)

# Append the header files here. this will get copied to include directory
set (VRPTOOLS_INCLUDE_FILES
    loadfromfiles.h
    vrptools.h
)

#add_library(baseclasses OBJECT ${BASECLASSES_SOURCES} ${BASECLASSES_INCLUDE_FILES} )
#add_library(basetrash OBJECT ${BASETRASH_SOURCES} ${BASETRASH_INCLUDE_FILES} )
#add_library(inittrash OBJECT ${INITTRASH_SOURCES} ${INITTRASH_INCLUDE_FILES} )
#add_library(solvertrash OBJECT ${SOLVERTRASH_SOURCES} ${SOLVERTRASH_INCLUDE_FILES} )

add_library (
    ${VRPTOOLS_LIBRARY_NAME}
    SHARED
    ${BASECLASSES_SOURCES} ${BASECLASSES_INCLUDE_FILES}
    ${BASETRASH_SOURCES} ${BASETRASH_INCLUDE_FILES}
    ${INITTRASH_SOURCES} ${INITTRASH_INCLUDE_FILES}
    ${SOLVERTRASH_SOURCES} ${SOLVERTRASH_INCLUDE_FILES}
    ${VRPTOOLS_SOURCES}
)
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CPP)
set_target_properties(${VRPTOOLS_LIBRARY_NAME} PROPERTIES VERSION ${VRPTOOLS_VERSION_STRING})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Threads_LIBRARIES} ${OSRM_LIBRARIES} ${GLOG_LIBRARIES} ${CURLPP_LIBRARIES} )


enable_testing()
add_subdirectory (tests)
