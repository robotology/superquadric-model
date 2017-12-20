# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author: Giulia Vezzani <giulia.vezzani@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

project(superquadric-model)

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpIDL)
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

set(headers   include/superquadric.h include/superqModule.h include/superqComputation.h include/superqVisualization.h)
set(sources src/superqModule.cpp  src/main.cpp src/superquadric.cpp src/superqComputation.cpp src/superqVisualization.cpp)
yarp_add_idl(IDL_GEN_FILES src/idl.thrift)
source_group("IDL Files" FILES src/idl.thrift)
source_group("DOC Files" FILES ${PROJECT_NAME}.xml)
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})

add_definitions(${YARP_DEFINES} ${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)

add_executable(${PROJECT_NAME} ${sources} ${headers} src/idl.thrift ${PROJECT_NAME}.xml ${IDL_GEN_FILES})
#add_executable(${PROJECT_NAME} ${sources} src/idl.thrift ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${IPOPT_LIBRARIES} ctrlLib)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
icubcontrib_add_uninstall_target()

# app and conf
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${PROJECT_NAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
file(GLOB database_box ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/primary_shape_database/box/*.txt)
yarp_install(FILES ${database_box} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/primary_shape_database/box)
file(GLOB database_sphere ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/primary_shape_database/sphere/*.txt)
yarp_install(FILES ${database_box} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/primary_shape_database/sphere)
file(GLOB database_cylinder ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/primary_shape_database/cylinder/*.txt)
yarp_install(FILES ${database_box} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/primary_shape_database/cylinder)
