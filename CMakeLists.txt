PROJECT(RAZOREDGE)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

ADD_SUBDIRECTORY(ulysses)# bin)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ulysses) # ${PROJECT_SOURCE_DIR}/3rdparty/g2o) # ${PROJECT_SOURCE_DIR}/3rdparty/include)
link_directories(${PROJECT_SOURCE_DIR}/lib) # ${PROJECT_SOURCE_DIR}/3rdparty/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#ADD_EXECUTABLE(PlaneLineSLAM ulysses/PlaneLineSLAM.cpp)
#target_link_libraries(PlaneLineSLAM systems evaluate)

#ADD_EXECUTABLE(Kinect2QualisysCalibration ulysses/Kinect2QualisysCalibration.cpp)
#target_link_libraries(Kinect2QualisysCalibration systems)

#ADD_EXECUTABLE(test ulysses/test.cpp)
#target_link_libraries(test map ${PCL_LIBRARIES})

#ADD_EXECUTABLE(simulate ulysses/Simulation.cpp)
#target_link_libraries(simulate systems)

#ADD_EXECUTABLE(slam_sim ulysses/SLAM_sim.cpp)
#target_link_libraries(slam_sim systems)

#ADD_EXECUTABLE(slam_tum ulysses/SLAM_tum.cpp)
#target_link_libraries(slam_tum systems)

ADD_EXECUTABLE(slam ulysses/main.cpp)
target_link_libraries(slam systems)
