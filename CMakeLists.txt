##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(motion_manager)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log_setup log thread filesystem)
include_directories( ${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS} )


##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui QApplication) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/motion_manager/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# External libraries
##############################################################################

# ----- Human-like Upper-limbs Motion Library (HUML) -------- #
set(HUML_HOME /home/gianpaolo/HUML)
include_directories(${HUML_HOME}/include)
add_library(HUML STATIC IMPORTED)
set_property(TARGET HUML PROPERTY IMPORTED_LOCATION ${HUML_HOME}/${CMAKE_BUILD_TYPE}/libHUML.a)

# -- IPOPT Solver
set(IPOPT_HOME /home/gianpaolo/CoinIpopt)
include_directories(
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/coin/ThirdParty
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/include/extra

)
link_directories(
    ${IPOPT_HOME}/${CMAKE_BUILD_TYPE}/lib
)

##############################################################################
# Binaries
##############################################################################
add_executable(motion_manager ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(motion_manager ${QT_LIBRARIES} ${catkin_LIBRARIES})

# ********** link external libraries ********
# Human-like Upper-limbs Motion Library (HUML)
target_link_libraries(motion_manager HUML)
# -- IPOPT Solver
target_link_libraries(motion_manager libipopt.a)
target_link_libraries(motion_manager libcoinmumps.a)
target_link_libraries(motion_manager libcoinlapack.a)
target_link_libraries(motion_manager libcoinasl.a)
target_link_libraries(motion_manager libcoinblas.a)
target_link_libraries(motion_manager libcoinmetis.a)
target_link_libraries(motion_manager libipoptamplinterface.a)
target_link_libraries(motion_manager dl) # this has to be the last one
target_link_libraries(motion_manager gfortran)# this has to be the last one

target_link_libraries(motion_manager ${Boost_LIBRARIES})


install(TARGETS motion_manager RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

