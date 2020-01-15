cmake_minimum_required(VERSION 2.8)
project( Hypertun_SR )

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS " -fopenmp -g -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse --std=c++11")
set(CMAKE_CXX_FLAGS "-fPIC -O3 -fopenmp -Wall -march=native -msse -msse2 -msse3 -mssse3 -msse4 -ffast-math -mfpmath=sse --std=c++11")

find_package( OpenCV 3 REQUIRED )
if (OpenCV_FOUND)
	message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
endif()

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# HPTSR lib
# add_definitions("-DHPTSR_DEBUG")
add_library(${PROJECT_NAME}
		src/sparse_stereo.cpp
		src/delaunay_triangulation.cpp
		src/disparity_interpolation.cpp
		src/cost_evaluation.cpp
		src/disparity_refinement.cpp
		src/support_resampling.cpp
		src/image_gradient.cpp
)
target_link_libraries(${PROJECT_NAME}
		${OpenCV_LIBS}
		sparsestereo
		triangulation
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test/bin)
# pipline test
add_executable( main.out
		test/main.cpp 
		test/pipeline.cpp
)
target_link_libraries( main.out 
		${PROJECT_NAME}
)
# rastertri test
add_executable(test_rastertri 
		test/test_rastertri.cc
		src/delaunay_triangulation.cpp
)
target_link_libraries(test_rastertri 
		${OpenCV_LIBS}
		triangulation
)
# triangulation test
add_executable(test_triangulation
		test/test_triangulation.cc
)
target_link_libraries(test_triangulation
		${OpenCV_LIBS}
		triangulation
)

add_subdirectory(ThirdParty/sparsestereo)
add_subdirectory(ThirdParty/triangulation)


