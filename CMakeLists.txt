# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Giulia Vezzani <giulia.vezzani@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

project(Superquadric-Detection)

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(OpenCV REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

find_package(IPOPT REQUIRED)

set(sources src/superquadric.cpp src/main.cpp)
source_group("Source Files" FILES ${sources})

include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS} iKin)

add_definitions(${YARP_DEFINES}) 
add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${IPOPT_LIBRARIES} ctrlLib)

add_subdirectory(app)

icubcontrib_add_uninstall_target()
