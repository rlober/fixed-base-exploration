# This file is part of ocra-yarp.
# Copyright (C) 2016 Institut des Systèmes Intelligents et de Robotique (ISIR)
# author(s): Ryan Lober, Antoine Hoarau
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.11)
PROJECT(fixed-base-exploration CXX)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(OcraRecipes REQUIRED)
find_package(OcraIcub REQUIRED)
find_package(SMLT REQUIRED)

list(APPEND CMAKE_MODULE_PATH
        ${YARP_MODULE_PATH}
        ${YCM_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules
     )

#add yarp definitions
add_definitions(${YARP_DEFINES}) #this contains also -D_REENTRANT

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)


FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/${PROJECT_NAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}/include
${OcraRecipes_INCLUDE_DIRS}
${OcraIcub_INCLUDE_DIRS}
${SMLT_INCLUDE_DIRS}
)

ADD_EXECUTABLE(${PROJECT_NAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(
${PROJECT_NAME}
${OcraIcub_LIBRARIES}
${SMLT_LIBRARIES}
# ${OcraRecipes_LIBRARIES}
# ${iDynTree_LIBRARIES}
# ${yarpWholeBodyInterface_LIBRARIES}
# ${Boost_LIBRARIES}
)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION bin)
