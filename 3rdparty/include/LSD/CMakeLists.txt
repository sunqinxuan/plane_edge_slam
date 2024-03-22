PROJECT(LSD)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

ADD_LIBRARY(LSD SHARED lsd.c)
target_link_libraries(LSD m)

ADD_EXECUTABLE(LSD_test lsd_cmd.c)
target_link_libraries(LSD_test LSD)

