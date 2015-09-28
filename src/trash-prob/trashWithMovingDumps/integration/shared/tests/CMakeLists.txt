include_directories(
    "../"
)

# Append the source files here
set (TEST001_SOURCES
     test001.cpp
)

# Append the header files here. this will get copied to include directory
set (TEST001_SOURCES_FILES

)

add_executable (test001 ${TEST001_SOURCES})
target_link_libraries (test001 LINK_PUBLIC ${VRPTOOLS_LIBRARY_NAME})
