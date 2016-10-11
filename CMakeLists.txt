##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
set(PROJECT_NAME motion_manager)
project(${PROJECT_NAME})

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
# Settings
##############################################################################

# ----------------- Path of the main directory -------------------------------- #
set(MAIN_PATH /home/gianpaolo/catkin_ws/src/motion_manager)

# configuration files
configure_file (
  "${PROJECT_SOURCE_DIR}/include/motion_manager/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/motion_manager/config.hpp"
)


## Tests
# TEST=0. do not run the tests
# TEST=1, run the tests
set(TEST 1)

## Documentation
# DOC=0. do not make the documentation
# DOC=1, make the documentation
set(DOC 1)

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
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# ********** link external libraries ********
# Human-like Upper-limbs Motion Library (HUML)
target_link_libraries(${PROJECT_NAME} HUML)
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

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


##############################################################################
# Testing
##############################################################################
if(TEST)

    set(PROJECT_NAME_LIB ${PROJECT_NAME}_lib)
    add_library(${PROJECT_NAME_LIB} STATIC ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
    target_link_libraries(${PROJECT_NAME_LIB} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    ##set_target_properties(${PROJECT_NAME_LIB} PROPERTIES POSITION_INDEPENDENT_CODE ON)


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

    ##target_link_libraries(${PROJECT_NAME_LIB} ${Boost_LIBRARIES})



    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    include_directories(${GTEST_INCLUDE_DIRS})
    enable_testing ()

    ## Test suite ros communication dialog
    set(PROJECT_TEST_NAME ${PROJECT_NAME}_test_ros_comm_dlg)
    file(GLOB TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/ros_comm_dlg/*.cpp)
    file(GLOB TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/*.hpp)
    add_executable(${PROJECT_TEST_NAME} ${TEST_CPP_FILES} ${TEST_HPP_FILES})
    target_link_libraries(${PROJECT_TEST_NAME} boost_system ${GTEST_LIBRARIES} ${PROJECT_NAME_LIB})
    ##target_link_libraries(${PROJECT_TEST_NAME} ${PROJECT_NAME_LIB})
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
        add_custom_target(doc COMMAND ${DOXYGEN_EXECUTABLE} "doxyconfig/doxyfile.ini"
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Building user's documentation into the doc directory ..."
        )
    endif()
endif()




