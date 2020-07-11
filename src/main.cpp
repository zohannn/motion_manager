/**
 * The motion manager
 *
 * @author: Gianpaolo Gulletta
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
#include <execinfo.h>
#include <signal.h>
#include "../include/motion_manager/main_application.hpp"
#include "../include/motion_manager/main_window.hpp"


// https://stackoverflow.com/questions/77005/how-to-automatically-generate-a-stacktrace-when-my-program-crashes
void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

/*****************************************************************************
** Main
*****************************************************************************/

/**
 * @brief This is the main function of the program
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    MainApplication app(argc, argv);
    motion_manager::MainWindow w(argc,argv);
    w.show();
    signal(SIGSEGV, handler); // handling segmentation fault
    signal(SIGABRT, handler); // handling abortion
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}




