#ifndef OBJECTIVECHART_H
#define OBJECTIVECHART_H

#include "ui_ObjectiveChart.h"
#include <QWidget>
#include <QtCharts>
#include <QChart>
#include <QLineSeries>

namespace Ui
{
class ObjectiveChart;
}

class ObjectiveChart : public QWidget
{
  Q_OBJECT

public:
  explicit ObjectiveChart(QWidget* parent = nullptr);
  ~ObjectiveChart();

public slots:

private:
  void setupChartAndChartView();

  QChart* chart_;
  QValueAxis* axisX_;
  QValueAxis* axisY_;
  QChartView* chartView_;
  Ui::ObjectiveChart* ui_;
};

#endif  // OBJECTIVECHART_H