cmake_minimum_required(VERSION 2.8)
project( SGBM )
find_package( OpenCV REQUIRED )
add_executable( main.out
				main.cpp 
				pipeline_sgbm.cpp )
target_link_libraries( main.out ${OpenCV_LIBS} )