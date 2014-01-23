# Copyright: (C) 2011-2013 Mikhail Frank, Leo Pape, Juxi Leitner
# Author: Mikhail Frank, Juxi Leitner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

CMAKE_MINIMUM_REQUIRED ( VERSION 2.6 )
SET (CMAKE_BUILD_TYPE Release)
PROJECT ( LEOGrasper )

SET ( LEOGrasper_VERSION_MAJOR 1 )
SET ( LEOGrasper_VERSION_MINOR 0 )

FIND_PACKAGE ( YARP REQUIRED )
INCLUDE_DIRECTORIES ( ${YARP_INCLUDE_DIRS} )

SET ( src_files	main.cpp
	hand_controller.cpp
)
SET ( hdr_files	hand_controller.h)

ADD_EXECUTABLE ( LEOGrasper ${hdr_files} ${src_files} )

TARGET_LINK_LIBRARIES ( LEOGrasper ${YARP_LIBRARIES} )

INSTALL ( TARGETS LEOGrasper DESTINATION ${CMAKE_SOURCE_DIR}/bin )

