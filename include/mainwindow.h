#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DExtras/QFirstPersonCameraController>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QPointLight>
#include <QApplication>
#include <QMainWindow>
#include <QObject>
#include <QtDataVisualization>
#include <QtCharts>
#include <QChart>
#include <QLineSeries>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

#include "bppinterface.h"
#include "BoxJsonParser.h"
#include "Box.h"
#include "Model/BoxSet.h"
#include "View/BinPackingView.h"
#include "ViewMgr/BinPackingViewMgr.h"
#include "opt/ParamEstimator.h"
#include "opt/ParticleFilter.h"
#include "View/DistributionView.h"
#include "ViewMgr/DistributionViewMgr.h"

using namespace QtDataVisualization;

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
  void resetBoxes();

protected:
  MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);

  void closeEvent(QCloseEvent* event);

  virtual void timerEvent(QTimerEvent* timerEvent);

  void doBinPacking();
  void doBinPacking(std::shared_ptr<bpa::Params>& params);

private slots:
  void on_resetButton_clicked();
  void on_loadButton_clicked();
  void on_planButton_clicked();
  void on_estimateButton_clicked();
  void on_deleteButton_clicked();

private:
  Ui::MainWindow* ui;

  static MainWindow* singleton_;

  Qt3DExtras::Qt3DWindow* view_;
  Qt3DRender::QCamera* camera_;
  Qt3DExtras::QOrbitCameraController* manipulator_;

  BoxSet* boxSet_;
  BinPackingView& binPackingView_;
  BinPackingViewMgr* binPackingViewMgr_;

  filter::ParticleFilter* particleFilter_;
  DistributionView& distributionView_;
  DistributionViewMgr* distributionViewMgr_;

  bpainf::BppInterface bpp_inf_;

  int timer_id_;
};

#endif  // MAINWINDOW_H
