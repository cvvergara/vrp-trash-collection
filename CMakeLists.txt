cmake_minimum_required (VERSION 3.2.2)

#https://cmake.org/cmake/help/v3.0/command/project.html

project(vrptc 0.0.0.1 LANGUAGES C CXX) 

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

set(VRPTC_LIBRARY "vrp-trash-collection")

set(DEPS_LIBRARY_NAME "deps")




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
    "src/initTrash"
    "src/baseClasses"
    "src/solverTrash"
    "src/baseTrash"
    )

# Activa c++14
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX0X)
message(STATUS "COMPILER_SUPPORTS_CXX0X= ${COMPILER_SUPPORTS_CXX0X}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CPP_FLAGS " -Wall ")
set(CMAKE_CPP_FLAGS_DEBUG " -Wall -g -Wall -Wextra -pedantic -Wuninitialized -Wunreachable-code -Wstrict-overflow=1 -D_FORTIFY_SOURCE=2")
set(CMAKE_CPP_FLAGS_RELEASE "")

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

set(OSRM_LIBRARIES /user/local/lib/libosrm.a)
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
#find_package(rapidjson REQUIRED)
##

### Includes
#include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${CURLPP_INCLUDE_DIRS})
#include_directories(${RAPIDJSON_INCLUDE_DIRS})
###

# base trash
file(GLOB UtilGlob
    src/baseClasses/*.cpp
    src/basseTrash/*.cpp
    src/initTrash/*.cpp
    src/solverTrash/*.cpp)



# Append the source files here
set (VRPTC_SOURCES
    loadfromfiles.cpp
    vrptools.cpp
    )

# Append the header files here. this will get copied to include directory
set (VRPTC_INCLUDE_FILES
    loadfromfiles.h
    vrptools.h
    )

#add_library(baseclasses OBJECT ${BASECLASSES_SOURCES} ${BASECLASSES_INCLUDE_FILES} )
#add_library(basetrash OBJECT ${BASETRASH_SOURCES} ${BASETRASH_INCLUDE_FILES} )
#add_library(inittrash OBJECT ${INITTRASH_SOURCES} ${INITTRASH_INCLUDE_FILES} )
#add_library(solvertrash OBJECT ${SOLVERTRASH_SOURCES} ${SOLVERTRASH_INCLUDE_FILES} )

add_library (
    ${VRPTC_LIBRARY_NAME}
    SHARED
    ${BASECLASSES_SOURCES} ${BASECLASSES_INCLUDE_FILES}
    ${BASETRASH_SOURCES} ${BASETRASH_INCLUDE_FILES}
    ${INITTRASH_SOURCES} ${INITTRASH_INCLUDE_FILES}
    ${SOLVERTRASH_SOURCES} ${SOLVERTRASH_INCLUDE_FILES}
    ${VRPTC_SOURCES}
    )
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CPP)
set_target_properties(${VRPTC_LIBRARY_NAME} PROPERTIES VERSION ${VRPTC_VERSION})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Threads_LIBRARIES} ${OSRM_LIBRARIES} ${GLOG_LIBRARIES} ${CURLPP_LIBRARIES} )


enable_testing()
add_subdirectory (tests)
