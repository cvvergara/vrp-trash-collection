cmake_minimum_required (VERSION 3.2.2)

#https://cmake.org/cmake/help/v3.0/command/project.html

project(vrptc VERSION 0.0.0.1 LANGUAGES C CXX) 


message ("Generating project ${PROJECT_NAME}")

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR AND NOT MSVC_IDE)
    message(FATAL_ERROR "\
In-source builds are not allowed. \
Please create a directory and run cmake from there, passing the path \
to this source directory as the last argument. \
This process created the file 'CMakeCache.txt' and the directory \
'CMakeFiles'. Please delete them.
")
endif()



list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(FindPkgConfig)

set(VRPTC_LIBRARY "vrp-trash-collection")

#set(DEPS_LIBRARY_NAME "deps")


option(DOVRPLOG "general debugging" ON)
option(DOSTATS "On for debugging" OFF)
option(VRPMINTRACE "Option Min Logging description not so much debugging" ON)
option(VRPMAXTRACE "Option Max Logging use for detailed debugging" OFF)

add_definitions(-DOSRMCLIENT)

if(DOVRPLOG)
    add_definitions(-DDOVRPLOG)
endif()

if(DOSTATS)
    add_definitions(-DDOSTATS)
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
    "include"
    )


#--------------------------
# c++ flags
#--------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS " -Wall ")
set(CMAKE_CXX_FLAGS_DEBUG " -Wall -g -Wall -Wextra -pedantic -Wuninitialized -fmax-errors=10 -Wunreachable-code -Wstrict-overflow=1 -D_FORTIFY_SOURCE=2")
set(CMAKE_CXX_FLAGS_RELEASE " ")

## ----------------------------------------
## Boost 
##    Needed for the osrmclient
## ----------------------------------------

set(BOOST_COMPONENTS date_time filesystem iostreams program_options regex system thread unit_test_framework)
set(BOOST_MIN_VERSION "1.49.0")


find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(NOT Boost_FOUND)
    MESSAGE(FATAL_ERROR "Fatal error: Boost (version >= 1.49.0) required.\n")
else()
    MESSAGE(STATUS "Boost_VERSION ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
endif()



##

## ----------------------------------------
## Threads
## ----------------------------------------

find_package(Threads REQUIRED)

##


## ----------------------------------------
## OSRM
## ----------------------------------------


pkg_check_modules(OSRM REQUIRED libosrm)
message(STATUS "OSRM_VERSION  ${OSRM_VERSION}")

if(NOT (OSRM_VERSION VERSION_EQUAL "v5.3.0"))
    message(FATAL_ERROR "Currently tested with OSRM v5.3.0")
endif()

#-----------------------------------------------------DELETE
message(STATUS "OSRM_LIBRARIES ${OSRM_LIBRARIES}") 
message(STATUS "OSRM_LIBRARY_DIRS ${OSRM_LIBRARY_DIRS}") 
message(STATUS "OSRM_INCLUDE_DIRS ${OSRM_INCLUDE_DIRS}") 
message(STATUS "OSRM_LDFLAGS ${OSRM_LDFLAGS}") 
include_directories(${OSRM_INCLUDE_DIRS})

mark_as_advanced(OSRM_INCLUDE_DIRS OSRM_LIBRARIES OSRM_LIBRARY_DIR)

## ----------------------------------------
## Glog
## ----------------------------------------

find_package(Glog REQUIRED)


##

## ----------------------------------------
## CURL
## ----------------------------------------

find_package(CURL REQUIRED)

##


#pkg_check_modules(CURLPP REQUIERED curlpp)
#include_directories(${CURLPP_INCLUDE_DIRS})



file(GLOB VRPTC_SOURCES
    src/osrm_connection/*.cpp
    src/nodes/*.cpp
    src/baseClasses/*.cpp
    src/baseTrash/*.cpp
    src/initTrash/*.cpp
    src/solverTrash/*.cpp
    src/integration/trash.cpp)


add_executable (
    ${PROJECT_NAME}
    ${VRPTC_SOURCES}
    )

#target_link_libraries (vrp-trash-collection LINK_PUBLIC ${PROJECT_NAME})

#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CPP)
#set_target_properties(${VRPTC_LIBRARY_NAME} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})

#message(FATAL_ERROR "got here--------------------------")
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${Threads_LIBRARIES} ${OSRM_LIBRARIES} ${GLOG_LIBRARIES} ${CURLPP_LIBRARIES} )


#enable_testing()
#add_subdirectory (tests)
