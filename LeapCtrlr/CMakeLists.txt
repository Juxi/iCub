# Copyright: (C) 2014 Juxi Leitner
# Author: Juxi Leitner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
 
PROJECT(LeapCtrlr)

# include(FindPkgConfig)
FIND_PACKAGE ( YARP REQUIRED )
Find_Package ( SDL REQUIRED )
Find_Package ( SDL_image REQUIRED )
 
ADD_EXECUTABLE(LeapCtrlr src/main.cpp
						src/sdl_helpers.cpp
						src/leap_helpers.cpp						
						src/yarp_helpers.cpp
						src/dot.cpp)


INCLUDE_DIRECTORIES(${SDL_INCLUDE_DIR} ${YARP_INCLUDE_DIRS} ./include/)


TARGET_LINK_LIBRARIES(LeapCtrlr	${SDL_LIBRARY}
								${SDLIMAGE_LIBRARY}
								${YARP_LIBRARIES}
#								${LEAP_LIBRARY}
								/Users/juxi/Code/iCub/LeapCtrlr/lib/libLeap.dylib
							)
