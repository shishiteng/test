cmake_minimum_required(VERSION 2.8.3)
project(visensor)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)

include_directories(include)


# library
add_library(visensor SHARED
	src/device.cpp 
	src/snTool.cpp)
target_link_libraries(visensor ${OpenCV_LIBRARIES} -lcyusb -lpthread)

# exe
add_executable(test src/test.cpp)
target_link_libraries(test 
	${OpenCV_LIBRARIES}
	${PROJECT_SOURCE_DIR}/build/libvisensor.so
 	-lcyusb -lpthread)

add_dependencies(test visensor)
