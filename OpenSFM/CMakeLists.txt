cmake_minimum_required(VERSION 2.8)
project( OpenSfM )
find_package( OpenCV REQUIRED )
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

ADD_DEFINITIONS(
	-std=c++11
)

add_executable( OpenSfM OpenSfM.cpp Demo.cpp )
target_link_libraries( OpenSfM ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})
