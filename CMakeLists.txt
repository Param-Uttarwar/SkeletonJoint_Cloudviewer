cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(Myskel)
 
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS} ${OPEN_NI_INCLUDE})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 

add_executable(Myskel mainV1.cpp)
 
target_link_libraries (Myskel ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OPEN_NI_LIB})
