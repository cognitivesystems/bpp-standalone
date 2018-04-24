#ifndef DISTRIBUTIONVIEW_H
#define DISTRIBUTIONVIEW_H

#include <QWidget>
#include <QtCharts>
#include <QChart>
#include <QLineSeries>
#include <boost/math/distributions/beta.hpp>

class DistributionView : public QWidget
{
  Q_OBJECT

public:
  explicit DistributionView(QWidget* parent = nullptr);
  QChartView* getChartView();

public slots:
  void onDistributionsUpdated(const std::vector<boost::math::beta_distribution<>> distributions);

private:
  void setupChartAndChartView();

  QChart* chart_;
  QChartView* chartView_;
};

#endif  // DISTRIBUTIONVIEW_H