#=========================================================================
#   Institute:   University of Minho
#                Dept. of Industrial Electronics
#
#   File:        CMakeLists.txt
#
#   Maintainer:  Gianpaolo Gulletta
#   Email:       gianpaologulletta@gmail.com;
#   Date:        2016 01 12
#
#   Description: The Motion Manager
#
#   Credits:
#==========================================================================

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
set(PROJECT_NAME motion_manager)
project(${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
)
include_directories(${catkin_INCLUDE_DIRS})


# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log_setup log thread filesystem)
include_directories( ${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS} )


find_package(OpenGL)
find_package(GLUT)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${QT_QTOPENGL_INCLUDE_DIR})


##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here


##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Settings
##############################################################################

## MoveIt! configuration
# MOVEIT=0. do not use the moveit planner
# MOVEIT=1, use the moveit planner
set (MOVEIT 1)

if(MOVEIT)
    find_package(aros_moveit_planner REQUIRED COMPONENTS
      humanoid_planner
    )
    include_directories(${aros_moveit_planner_INCLUDE_DIRS})
endif()

## Humanoid parts configuration
# 1 means that the part is present and it is taken into account for the generation of the trajectories,
# 0 otherwise.
# The torso and the arms are the only parts assumed for any humanoid robot. In other words, the anthropomorphism of the system is assumed.
# The characteristics of the torso will be those ones of the humanoid.
# However the following parts may be taken into account when they are present
# set (HEAD 0)
# set (NECK 0)
# set (PELVIS 1)
# set (RIGHT_UPPER_LEG 1)
# set (RIGHT_LOWER_LEG 0)
# set (RIGHT_FOOT 0)
# set (LEFT_UPPER_LEG 1)
# set (LEFT_LOWER_LEG 0)
# set (LEFT_FOOT 0)

## Hand configuration
# HAND=0. Human hand
# HAND=1, Barrett hand
set (HAND 1)

set(MAIN_PATH ${PROJECT_SOURCE_DIR}) ## Path of the main directory
set(PATH_SCENARIOS /home/$ENV{USER}/scenarios) ## Path of the scenarios

# configuration files
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp"
)


## Tests
# TEST=0. do not run the tests
# TEST=1, run the tests
set(TEST 0)

## Documentation
# DOC=0. do not make the documentation
# DOC=1, make the documentation
set(DOC 0)


##############################################################################
# External libraries
##############################################################################

# ----- Human-like Upper-limbs Motion Planner (HUMP) -------- #
set(HUMP_HOME /home/$ENV{USER}/HUMP)
include_directories(${HUMP_HOME}/include)
add_library(HUMP STATIC IMPORTED)
set_property(TARGET HUMP PROPERTY IMPORTED_LOCATION ${HUMP_HOME}/${CMAKE_BUILD_TYPE}/libHUMP.a)

# -- IPOPT Solver
set(IPOPT_HOME /home/$ENV{USER}/CoinIpopt)
include_directories(
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin/ThirdParty
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/extra

)
link_directories(
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/lib
)

# -- QCustomPlot
add_definitions(-DQCUSTOMPLOT_USE_LIBRARY)
set(QCUSTOMPLOT_HOME /home/$ENV{USER}/qcustomplot)
include_directories(
    ${QCUSTOMPLOT_HOME}/${CMAKE_BUILD_TYPE}/include
)
link_directories(
    ${QCUSTOMPLOT_HOME}/${CMAKE_BUILD_TYPE}/lib
)

# -- QwtPlot3D
set(QWTPLOT3D_HOME /home/$ENV{USER}/qwtplot3d)
include_directories(
    ${QWTPLOT3D_HOME}/include
)
link_directories(
    ${QWTPLOT3D_HOME}/${CMAKE_BUILD_TYPE}/lib
)

# -- QCPDocumentObject
set(QCPDOCUMENTOBJECT_HOME /home/$ENV{USER}/qcpdocumentobject)
include_directories(
    ${QCPDOCUMENTOBJECT_HOME}/${CMAKE_BUILD_TYPE}/include
)
link_directories(
    ${QCPDOCUMENTOBJECT_HOME}/${CMAKE_BUILD_TYPE}/lib
)

# -- EigenPCA
set(EIGENPCA_HOME /home/$ENV{USER}/EigenPCA)
include_directories(
    ${EIGENPCA_HOME}/${CMAKE_BUILD_TYPE}/include
)
link_directories(
    ${EIGENPCA_HOME}/${CMAKE_BUILD_TYPE}/lib
)

# -- Persistence1D
set(PERSISTENCE1D_HOME /home/$ENV{USER}/Persistence1D)
include_directories(
    ${PERSISTENCE1D_HOME}/${CMAKE_BUILD_TYPE}/include
)
link_directories(
    ${PERSISTENCE1D_HOME}/${CMAKE_BUILD_TYPE}/lib
)

##############################################################################
# Binaries
##############################################################################
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# ********** link external libraries ********
# Human-like Upper-limbs Motion Planner (HUMP)
target_link_libraries(${PROJECT_NAME} HUMP)

if(MOVEIT)
    # Humanoid moveit planner Library
    target_link_libraries(${PROJECT_NAME} humanoid_planner)
endif()

# -- IPOPT Solver
target_link_libraries(${PROJECT_NAME} libipopt.a)
target_link_libraries(${PROJECT_NAME} libcoinmumps.a)
target_link_libraries(${PROJECT_NAME} libcoinlapack.a)
target_link_libraries(${PROJECT_NAME} libcoinasl.a)
target_link_libraries(${PROJECT_NAME} libcoinblas.a)
target_link_libraries(${PROJECT_NAME} libcoinmetis.a)
target_link_libraries(${PROJECT_NAME} libipoptamplinterface.a)
target_link_libraries(${PROJECT_NAME} dl) # this has to be the last one
target_link_libraries(${PROJECT_NAME} gfortran)# this has to be the last one

# -- QCustomPlot and QCPDocumentObject
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(${PROJECT_NAME} qcustomplotd qcpdocumentobjectd)
else()
    target_link_libraries(${PROJECT_NAME} qcustomplot qcpdocumentobject)
endif()

# -- QwtPlot3D
target_link_libraries(${PROJECT_NAME} qwtplot3d)
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${QT_QTOPENGL_LIBRARY} GLU)



# -- EigenPCA
target_link_libraries(${PROJECT_NAME} EigenPCA)

# -- Persistence1D
target_link_libraries(${PROJECT_NAME} Persistence1D)



##############################################################################
# Testing
##############################################################################
if(TEST)

    set(PROJECT_NAME_LIB ${PROJECT_NAME}_lib)
    add_library(${PROJECT_NAME_LIB} STATIC ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
    target_link_libraries(${PROJECT_NAME_LIB} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


    # ********** link external libraries ********
    # Human-like Upper-limbs Motion Library (HUML)
    target_link_libraries(${PROJECT_NAME_LIB} HUML)
    # -- IPOPT Solver
    target_link_libraries(${PROJECT_NAME_LIB} libipopt.a)
    target_link_libraries(${PROJECT_NAME_LIB} libcoinmumps.a)
    target_link_libraries(${PROJECT_NAME_LIB} libcoinlapack.a)
    target_link_libraries(${PROJECT_NAME_LIB} libcoinasl.a)
    target_link_libraries(${PROJECT_NAME_LIB} libcoinblas.a)
    target_link_libraries(${PROJECT_NAME_LIB} libcoinmetis.a)
    target_link_libraries(${PROJECT_NAME_LIB} libipoptamplinterface.a)
    target_link_libraries(${PROJECT_NAME_LIB} dl) # this has to be the last one
    target_link_libraries(${PROJECT_NAME_LIB} gfortran)# this has to be the last one
    # -- QCustomPlot and QCPDocumentObject
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_link_libraries(${PROJECT_NAME} qcustomplotd qcpdocumentobjectd)
    else()
        target_link_libraries(${PROJECT_NAME} qcustomplot qcpdocumentobject)
    endif()
    # -- QwtPlot3D
    target_link_libraries(${PROJECT_NAME} qwtplot3d)
    # -- EigenPCA
    target_link_libraries(${PROJECT_NAME} EigenPCA)

    # -- Persistence1D
    target_link_libraries(${PROJECT_NAME} Persistence1D)

    enable_testing ()

    ## Test suite ros communication dialog
    set(CMAKE_AUTOMOC ON)
    set(PROJECT_TEST_NAME ${PROJECT_NAME}_test_ros_comm_dlg)
    file(GLOB TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/ros_comm_dlg/*.cpp)
    file(GLOB TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/*.hpp)
    add_executable(${PROJECT_TEST_NAME} ${TEST_CPP_FILES} ${TEST_HPP_FILES})
    SET( TEST_LIBRARIES ${QT_QTTEST_LIBRARY} ${QT_LIBRARIES} )
    target_link_libraries(${PROJECT_TEST_NAME} ${TEST_LIBRARIES} ${PROJECT_NAME_LIB})
    add_test (NAME Test_ros_comm_dlg COMMAND ${PROJECT_TEST_NAME})


endif()
##############################################################################
# Documenting
##############################################################################
if(DOC)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(WARNING "Doxygen not found, unable to generate documentation")
    else()
        add_custom_target(doc_${PROJECT_NAME} COMMAND ${DOXYGEN_EXECUTABLE} "doxyconfig/doxyfile.ini"
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Building user's documentation into the doc directory ..."
        )
    endif()
endif()
##############################################################################
# Installing
##############################################################################
install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




