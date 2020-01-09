cmake_minimum_required(VERSION 2.8)
project(TEST_HPTSR)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
find_package(OpenCV 3 REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ../libs/
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(test_triangulation
    test_triangulation.cc
    ../libs/triangulation/DelaunayTriangulation.cpp
)
target_link_libraries(test_triangulation
    ${OpenCV_LIBS}
)