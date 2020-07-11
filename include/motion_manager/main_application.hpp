#include <QApplication>
#include <exception>
#include <iostream>


class MainApplication : public QApplication {
public:
    MainApplication(int& argc, char** argv);
    bool notify(QObject* receiver, QEvent* event); 

};
