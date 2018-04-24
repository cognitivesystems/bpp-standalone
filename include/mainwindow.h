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

  std::vector<double> linspace(double a, double b, int n)
  {
    std::vector<double> array;
    double step = (b - a) / (n - 1);

    while (a <= b)
    {
      array.push_back(a);
      a += step;  // could recode to better handle rounding errors
    }
    return array;
  }

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

  bpainf::BppInterface bpp_inf_;

  int timer_id_;

  // visualisation
  std::vector<double> x_target;
  std::vector<double> y_target;

  QLineSeries* series0;
  QLineSeries* series1;
  QLineSeries* series2;
  QLineSeries* series3;
  QLineSeries* series4;

  QChart* chart;

  QChartView* chartView;

  std::vector<double> pdf0;
  std::vector<double> pdf1;
  std::vector<double> pdf2;
  std::vector<double> pdf3;
  std::vector<double> pdf4;

  QMainWindow window;
};

#endif  // MAINWINDOW_H
