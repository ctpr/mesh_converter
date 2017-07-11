cmake_minimum_required(VERSION 2.6)
project(mesh_converter)

SET(CMAKE_VERBOSE_MAKEFILE ON)

#we might have to set the OpenCV_DIR (!)
#set(CMAKE_MODULE_PATH extern/cmake-modules ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/extern/cmake-modules)
set(CMAKE_COLOR_MAKEFILE ON)

###########
## BOOST ##
###########

#boost is just required ...
find_package(Boost REQUIRED)
if(Boost_FOUND)
    add_definitions("-DHAS_BOOST")
endif()


#############
## libMesh ##
#############

find_path(LIBMESH_INCLUDE_DIR base/libmesh_base.h
          PATH_SUFFIXES libmesh )

find_library(LIBMESH_LIBRARY NAMES libmesh
             )

set(LIBMESH_LIBRARIES ${LIBMESH_LIBRARY} )
set(LIBMESH_INCLUDE_DIRS ${LIBMESH_INCLUDE_DIR} )

##############################
## Qt, VTK, GLUT, GLEW, ... ##
##############################

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#find_package(OpenCV)

find_package(PCL)
add_definitions(
    ${PCL_DEFINITIONS}
)

if(APPLE)
    set(CMAKE_CXX_LINK_FLAGS "-framework OpenGL")
endif () #APPLE

	
include_directories(
	./
    ${PROJECT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
	${LIBMESH_INCLUDE_DIRS}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
)

#set(CMAKE_CXX_FLAGS -Wall -W -pedantic -std=c99)

set(COMMON_HEADERS
    MeshConverter.hpp
)

set(COMMON_SOURCES
    MeshConverter.cpp
)

set(MAIN_APPLICATION
    main.cpp
)

add_executable(mesh_converter ${MAIN_APPLICATION} ${COMMON_SOURCES} ${COMMON_HEADERS})

target_link_libraries(mesh_converter
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${VTK_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
#        ${PCL_FILTERS_LIBRARIES}
#        ${PCL_FEATURES_LIBRARIES}
    ${PCL_GEOMETRY_LIBRARIES}
    ${PCL_SURFACE_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
	${LIBMESH_LIBRARIES}
)

