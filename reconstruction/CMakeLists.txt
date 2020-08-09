cmake_minimum_required(VERSION 2.8.11)
#Project
project(ProcessMesh)

find_package( OpenCV )

include_directories(${OpenCV_INCLUDE_DIRS})
		

#Output
add_executable(
	ProcessMesh
	new_enc.cpp
	util.hpp
)

target_link_libraries(ProcessMesh ${OpenCV_LIBS})
