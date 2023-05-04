cmake_minimum_required(VERSION 2.8)

project(I_Railway)

add_executable(Railway.out I_Railway/Railway.cpp I_Railway/RailModels.cpp)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS})

target_link_libraries( Railway.out ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
