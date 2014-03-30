# Copyright: (C) 2014 Juxi Leitner
# Author: Juxi Leitner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
 
PROJECT(SDLCtrlr)

# include(FindPkgConfig)
FIND_PACKAGE ( YARP REQUIRED )
Find_Package (SDL REQUIRED)
Find_Package ( SDL_image REQUIRED )

 
ADD_EXECUTABLE(sdlctrlr src/main.cpp
						src/sdl_helpers.cpp
						src/sixaxis_helpers.cpp						
						src/yarp_helpers.cpp
						src/dot.cpp)


INCLUDE_DIRECTORIES(${SDL_INCLUDE_DIR}  ${YARP_INCLUDE_DIRS} )
TARGET_LINK_LIBRARIES(sdlctrlr ${SDL_LIBRARY} ${SDLIMAGE_LIBRARY} ${YARP_LIBRARIES} )
