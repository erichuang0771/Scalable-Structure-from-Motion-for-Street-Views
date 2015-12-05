cmake_minimum_required(VERSION 2.8)
project( OpenSfM )
find_package( OpenCV REQUIRED )
find_package(Armadillo REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

include_directories(${ARMADILLO_INCLUDE_DIRS})

ADD_DEFINITIONS(
	-std=c++11
)

add_executable( OpenSfM BA.cc GeometryFunctions.cpp utils.cpp OpenSfM.cpp Demo.cpp )
target_link_libraries( OpenSfM ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES} ${CERES_LIBRARIES})
