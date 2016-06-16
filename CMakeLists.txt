cmake_minimum_required( VERSION 2.8 )
project( zed_slam )

if ( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE "Release")
    #set(CMAKE_BUILD_TYPE "Debug")
endif()

# set( CMAKE_CXX_COMPILER "clang++" )
set( CMAKE_CXX_FLAGS "-Wall -O3 -pthread -Wno-sign-compare -Wno-reorder -Wno-switch -Wno-deprecated-register" )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# For SEE and c++ std
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

find_package( OpenCV 2.4 REQUIRED )
find_package( Eigen3 3.1.0 REQUIRED )
find_package( ZED 0.9 REQUIRED )
find_package( CUDA REQUIRED )
find_package( fast REQUIRED )
find_package( G2O REQUIRED )
find_package( Cholmod REQUIRED ) 
find_package( Pangolin REQUIRED )
# find_package( Sophus REQUIRED )

include_directories( 
    ${PROJECT_SOURCE_DIR}/include    
    ${PROJECT_SOURCE_DIR}/3rdparty
    ${PROJECT_SOURCE_DIR}/3rdparty/vikit_common/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${fast_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIR}
    #${Sophus_INCLUDE_DIRS}
)

set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib )

link_directories( 
    ${CUDA_LIBRARIES_DIRS}
    ${ZED_LIBRARY_DIR}
    ${OpenCV_LIBRARY_DIRS}
    ${fast_LIBRARIES}
)

# the library
add_subdirectory( src )
# tests
add_subdirectory( test )
