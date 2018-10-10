cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(processing_sem3d)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#Include the FindCUDA script
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable (icp_demo 
src/main.cpp
src/icp/icp.cpp)
target_link_libraries (icp_demo ${PCL_LIBRARIES})

TARGET_LINK_LIBRARIES (icp_demo LINK_PUBLIC ${Boost_LIBRARIES})

SET(CMAKE_CXX_FLAGS "-std=c++0x")