#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>

#include "mainwindow.h"
#include "scenerenderer3d.h"
#include "ui_mainwindow.h"
#include <iostream>

#include "bppinterface.h"
#include <box_factory/BoxJsonParser.h>
#include <bpa/Box.h>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  virtual ~MainWindow();
  static MainWindow* instance();

  void resetCamera();

protected:
  MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);

  void closeEvent(QCloseEvent* event);

  virtual void timerEvent(QTimerEvent* timerEvent);

private slots:
  void on_resetButton_clicked();
  void on_loadButton_clicked();
  void on_planButton_clicked();
  void on_deleteButton_clicked();

private:
  Ui::MainWindow* ui;

  static MainWindow* singleton_;

  Qt3DExtras::Qt3DWindow* view_;
  Qt3DRender::QCamera* camera_;
  Qt3DExtras::QFirstPersonCameraController* manipulator_;

  SceneRenderer3D* scene_3d_;

  std::vector<bpa::Box> boxes_;
  std::vector<bpa::Box> planned_boxes_;

  std::vector<ObjectModel> models_;

  bpainf::BppInterface bpp_inf_;

  int timer_id_;
};

#endif  // MAINWINDOW_H
