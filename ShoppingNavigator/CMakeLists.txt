# The MIT License (MIT)
#
#  Copyright (c) 2014 Carnegie Mellon University
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#  THE SOFTWARE.

cmake_minimum_required(VERSION 2.4)

PROJECT(ImageServer)

# MOOS source dir
SET(MOOS_SOURCE_DIR "moos/MOOS")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# MOOS build dir
SET( MOOS_DIR "moos/MOOS")
SET( MOOS_INCLUDE_DIRS "moos/MOOS/Essentials" "moos/MOOS/Core")
SET( MOOS_LIBRARY_DIRS "moos/MOOS/MOOSBin")

IF( WIN32 )
    IF( MSVC )
        ADD_DEFINITIONS(-DWINDOWS_NT 
                -D_CRT_SECURE_NO_DEPRECATE -D_SCL_SECURE_NO_DEPRECATE)
        SET(PLATFORM_LIBS wsock32 comctl32)
    ENDIF( MSVC )
ENDIF( WIN32 )

IF( UNIX )
    ADD_DEFINITIONS(-DUNIX)
    SET(PLATFORM_LIBS m pthread)
    IF("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
        ADD_DEFINITIONS( -DPLATFORM_DARWIN )
        SET( PLATFORM_DARWIN 1 ) 
    ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    IF("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
        ADD_DEFINITIONS( -DPLATFORM_LINUX )
        SET( PLATFORM_LINUX 1 ) 
    ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
    IF("${CMAKE_SYSTEM_NAME}" MATCHES "SunOS")
        ADD_DEFINITIONS( -DPLATFORM_SUNOS )
        SET( PLATFORM_SUNOS 1 ) 
        SET(PLATFORM_LIBS ${PLATFORM_LIBS} socket nsl rt)
    ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "SunOS")	
ENDIF( UNIX )

FIND_PACKAGE(MOOS REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} "/usr/include/flycapture" "${PROJECT_SOURCE_DIR}/..")

SET(IMAGE_SOURCES
  source/image/ImageServer.cpp
  source/image/ImageMap.cpp
  source/image/CameraCapture.cpp
  source/image/UdpServer.cpp
)

SET(INERTIAL_SOURCES
  source/inertial/InertialTracker.cpp
  #source/inertial/cpdr.cpp
)

SET(BASIL_DIRS /usr/local/include/basil-2.3/)
SET(BASIL_LIB /usr/local/lib/libbasil.so)

LINK_DIRECTORIES("xsens-mti/bin")

message( STATUS "MOOS LIBRARY DIRS: ${MOOS_LIBRARY_DIRS}" )
message( STATUS "MOOS LIBRARIES: ${MOOS_LIBRARIES}" )

LINK_DIRECTORIES("/usr/lib" ${MOOS_LIBRARY_DIRS} "${PROJECT_SOURCE_DIR}/binary")
ADD_EXECUTABLE(ImageServer ${IMAGE_SOURCES})
SET_TARGET_PROPERTIES(ImageServer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/binary)
TARGET_LINK_LIBRARIES(ImageServer MOOS MOOSGen ${PLATFORM_LIBS} ${OpenCV_LIBS} flycapture)

ADD_EXECUTABLE(InertialTracker ${INERTIAL_SOURCES})
INCLUDE_DIRECTORIES(InertialTracker ${BASIL_DIRS})
SET_TARGET_PROPERTIES(InertialTracker PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/binary)
TARGET_LINK_LIBRARIES(InertialTracker cmt rt ${BASIL_LIB} boost_thread boost_system)
