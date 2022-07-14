cmake_minimum_required(VERSION 3.5)
project(ObjDetector)
message(=============================================================)
message("-- Ready to compile ObjDetector")
message(=============================================================)

#========================
#  Boost
#========================
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

find_package(Boost COMPONENTS system date_time regex REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND EXTERNAL_LIBS ${Boost_LIBRARIES})
list(APPEND EXTERNAL_LIBS "-lpthread")

# fix pthread missing on ubuntu18.04 or ubuntu20.04
if(WIN32)
else()
  list(APPEND EXTERNAL_LIBS pthread)
endif(WIN32)

#========================
#========================

include(FindPkgConfig)
find_package(Boost 1.60.0 COMPONENTS filesystem system REQUIRED)

message(STATUS "Start ..")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLIDAR_VISUALIZATION_ON -pthread -std=c++0x -Wpointer-arith -Wl,--no-as-needed")

add_definitions(-std=c++11)
add_definitions(-std=c++14)
set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(rs_driver REQUIRED)
#include_directories(${DRIVER_INCLUDE_DIRS})
include_directories(
	/usr/include
 	/usr/include/pcl-1.8
	/usr/include/eigen3
	/usr/include/vtk-7.1/
	src
	lib
	${DRIVER_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${rs_driver_INCLUDE_DIRS}
)
link_directories(
	/usr/local/lib
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)
file(GLOB DH_SRC_FILE src/*.cpp lib/*.cpp)

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED OpenCV 4.1 REQUIRED)
find_package(OpenCV 4.1 REQUIRED)
add_definitions(
	${PCL_DEFINITIONS}
	${OpenCV_DEFINITIONS}
)
#include_directories(${PCL_INCLUDE_DIRS})

add_executable(ObjDetector
              ObjDetector.cpp
	      	  ${DH_SRC_FILE}
              )
target_link_libraries(ObjDetector
                    pthread pcl_common pcl_io pcl_search pcl_filters pcl_segmentation pcl_visualization vtkCommonCore-7.1 boost_system  
                    ${EXTERNAL_LIBS} 
		    ${PCL_LIBRARIES}
	            ${OpenCV_LIBRARIES}
		    ${rs_driver_LIBRARIES}         
)
