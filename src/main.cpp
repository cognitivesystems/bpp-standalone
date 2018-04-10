#include "mainwindow.h"
#include <QApplication>

MainWindow* MainWindow::singleton_ = NULL;
bpa::Params* bpa::Params::singleton_ = NULL;

int main(int argc, char* argv[])
{
  QApplication app(argc, argv);

  QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  MainWindow::instance()->show();

  return app.exec();
}
