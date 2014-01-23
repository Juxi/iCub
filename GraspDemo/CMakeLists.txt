# Copyright: (C) 2013-2014 Juxi Leitner
# Author: Mikhail Frank, Juxi Leitner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED ( VERSION 2.6 )
SET (CMAKE_BUILD_TYPE Release)
PROJECT ( LEOGrasper )

SET ( GraspDemo_VERSION_MAJOR 1 )
SET ( GraspDemo_VERSION_MINOR 0 )

FIND_PACKAGE ( YARP REQUIRED )
INCLUDE_DIRECTORIES ( ${YARP_INCLUDE_DIRS} )

SET ( src_files	main.cpp )

ADD_EXECUTABLE ( GraspDemo ${src_files} )

TARGET_LINK_LIBRARIES ( GraspDemo ${YARP_LIBRARIES} )

INSTALL ( TARGETS GraspDemo DESTINATION ${CMAKE_SOURCE_DIR}/bin )

