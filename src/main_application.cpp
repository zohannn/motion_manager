#include <execinfo.h>
#include <unistd.h>
#include <stdexcept>
#include "../include/motion_manager/main_application.hpp"


MainApplication::MainApplication(int& argc, char** argv) :
    QApplication(argc, argv) {}

bool MainApplication::notify(QObject* receiver, QEvent* event)
{
    try {
        return QApplication::notify(receiver, event);
    } catch (std::exception &e) {
        qFatal("Error %s sending event %s to object %s (%s)",
            e.what(), typeid(*event).name(), qPrintable(receiver->objectName()),
            typeid(*receiver).name());
    } catch (...) {
        qFatal("Error <unknown> sending event %s to object %s (%s)",
            typeid(*event).name(), qPrintable(receiver->objectName()),
            typeid(*receiver).name());
    }
}

