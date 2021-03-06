cmake_minimum_required (VERSION 2.6)

project(slamPoc)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall -Wno-psabi -pthread")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} src/frame src/pose)
link_directories(${OpenCV_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS} ${Boost_DEFINITIONS})

# Executable for slamPoc
add_executable(slamPoc src/campose_caminput_main.cpp 
			                  src/pose/KeypointProcessorGpu.cpp src/pose/CameraPoseEstimator.cpp src/pose/triangulate.cpp)
target_link_libraries(slamPoc ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} pthread boost_system)
