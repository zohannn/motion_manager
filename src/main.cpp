/**
 * Motion Planner
 *
 * @author: Gianpaolo Gulletta d6468@dei.uminho.pt
 *
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date December 2015
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QApplication>
#include "../include/MotionPlanner/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    MotionPlanner::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}




