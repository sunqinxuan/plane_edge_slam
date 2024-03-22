
SET(SRC_LIST_MAP types/map.cpp types/types.cpp)
SET(SRC_LIST_FEAT features/plane_segmentation.cpp features/plane_fitting.cpp 
	features/it_geometric_feature_matching.cpp features/edge_extraction.cpp
	features/plane_extraction.cpp)
	#features/geometric_feature_matching.cpp 
    #features/plane_extraction_base.cpp    features/line_extraction.cpp) #features/plane_extraction.cpp
SET(SRC_LIST_MOTION motion/motion_estimation.cpp)
#SET(SRC_LIST_EVAL evaluate/relative_pose_error.cpp)
SET(SRC_LIST_TRAJ trajectory/traj_generation.cpp trajectory/traj_puzzle.cpp)
SET(SRC_LIST_SYSTEMS systems/plane_edge_SLAM.cpp)
#systems/system_simulation.cpp systems/system_slam_sim.cpp  systems/system_relocalization.cpp)
#		 systems/system_extrinsic_calibration.cpp systems/system_plane_line_shadow.cpp

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ulysses) # ${PROJECT_SOURCE_DIR}/3rdparty/g2o)
# ${PROJECT_SOURCE_DIR}/3rdparty/include)
#link_directories(${PROJECT_SOURCE_DIR}/3rdparty/lib)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR}) 
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})
find_package(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS serialization filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#find_package(Pangolin REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS}) 
#set(G2O_LIBS g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension)

#message(STATUS "G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIRS}")


ADD_LIBRARY(map SHARED ${SRC_LIST_MAP})
target_link_libraries(map ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_LIBRARY(features SHARED ${SRC_LIST_FEAT})
target_link_libraries(features map) # LSD LineMatchingLib ${PCL_LIBRARIES} ${OpenCV_LIBS})

ADD_LIBRARY(traj SHARED ${SRC_LIST_TRAJ})
target_link_libraries(traj map)

ADD_LIBRARY(motion ${SRC_LIST_MOTION})
target_link_libraries(motion map ${CERES_LIBRARIES})# ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/libg2o.so) #${G2O_LIBS}) #

#ADD_LIBRARY(evaluate SHARED ${SRC_LIST_EVAL})

ADD_LIBRARY(systems ${SRC_LIST_SYSTEMS} ${Boost_LIBRARIES})
target_link_libraries(systems features traj motion) # evaluate )



