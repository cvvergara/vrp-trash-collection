include_directories(
    "../"
)

## ----------------------------------------
# Append the source files here
set (TEST001_SOURCES
     test001.cpp
)

# Append the header files here. this will get copied to include directory
set (TEST001_INCLUDE_FILES

)

add_executable (test001 ${TEST001_SOURCES})
target_link_libraries (test001 LINK_PUBLIC ${VRPTOOLS_LIBRARY_NAME})

## ----------------------------------------
# Append the source files here
set (TEST002_SOURCES
    ../../../../../baseClasses/basictypes.cpp
    ../../../../../baseClasses/osrmclient.cpp
    ../../../../../baseClasses/node.cpp
    ../../../../../baseClasses/stats.cpp
    ../../../../../baseClasses/vrp_assert.cpp
    test002.cpp
    #../../../../../baseClasses/basictypes.h
    #./../../../../baseClasses/logger.h

)

# Activa c++11
set(CMAKE_CPP_FLAGS "${CMAKE_CPP_FLAGS} ${RAPIDJSON_CXX_FLAGS}")

# Append the header files here. this will get copied to include directory
set (TEST002_INCLUDE_FILES

)

MESSAGE(STATUS "Boost: " ${Boost_LIBRARIES} )
MESSAGE(STATUS "OSRM: " ${OSRM_LIBRARIES} )
MESSAGE(STATUS "RapidJSON: " ${RAPIDJSON_CXX_FLAGS} )
MESSAGE(STATUS "glog: " ${GLOG_LIBRARIES} )
MESSAGE(STATUS "curlpp: " ${CURLPP_LIBRARIES} )


add_executable (test002 ${TEST002_SOURCES})
target_link_libraries (test002 ${OSRM_LIBRARIES} ${GLOG_LIBRARIES} ${CURLPP_LIBRARIES} ${Boost_LIBRARIES} pthread rt )
#target_link_libraries (test002 ${OSRM_LIBRARIES} ${GLOG_LIBRARIES} ${CURLPP_LIBRARIES} ${Boost_LIBRARIES} )

## ----------------------------------------
set (TESTPOINT_SOURCES
    testpoint.cpp
)
set (TESTPOINT_INCLUDE_FILES

)
add_executable (testPoint ${TESTPOINT_SOURCES})
target_link_libraries (testPoint ${Boost_LIBRARIES})

## ----------------------------------------
set (TESTPHANTOMNODE_SOURCES
    ../../../../../baseClasses/phantomnode.cpp
    testphantomnode.cpp
)
set (TESTPHANTOMNODE_INCLUDE_FILES

)
add_executable (testPhantomNode ${TESTPHANTOMNODE_SOURCES})
target_link_libraries (testPhantomNode ${Boost_LIBRARIES})
