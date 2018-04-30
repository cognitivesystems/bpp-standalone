#ifndef DISTRIBUTIONCHART_H
#define DISTRIBUTIONCHART_H

#include "ui_DistributionChart.h"
#include <QWidget>
#include <QtCharts>
#include <QChart>
#include <QLineSeries>
#include <boost/math/distributions/beta.hpp>

namespace Ui
{
class DistributionChart;
}

class DistributionChart : public QWidget
{
  Q_OBJECT

public:
  explicit DistributionChart(QWidget* parent = nullptr);
  ~DistributionChart();

public slots:
  void onDistributionsUpdated(const std::vector<boost::math::beta_distribution<>> distributions);

private:
  void setupChartAndChartView();

  QChart* chart_;
  QValueAxis* axisX_;
  QValueAxis* axisY_;
  QChartView* chartView_;
  Ui::DistributionChart* ui_;
};

#endif  // DISTRIBUTIONCHART_H