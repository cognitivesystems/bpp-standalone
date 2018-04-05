#include <QApplication>
#include "mainwindow.h"

MainWindow* MainWindow::singleton_ = NULL;

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    MainWindow::instance()->show();

    return app.exec();
}
