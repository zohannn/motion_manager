#include "../include/motion_manager/main_application.hpp"
#include <exception>
#include <iostream>

MainApplication::MainApplication(int& argc, char** argv) :
    QApplication(argc, argv) {}

bool MainApplication::notify(QObject* receiver, QEvent* event) {
    bool done = true;
    try {
        done = QApplication::notify(receiver, event);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    } catch (const std::runtime_error &ex) {
        std::cerr << ex.what() << std::endl;
    }
    return done;
} 
