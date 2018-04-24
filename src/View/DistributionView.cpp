#include "View/DistributionView.h"
#include "MathUtils.h"

DistributionView::DistributionView(QWidget* parent)
  : QWidget(parent), chart_(new QChart()), chartView_(new QChartView(chart_, this))
{
  setupChartAndChartView();
}

QChartView* DistributionView::getChartView()
{
  return chartView_;
}

void DistributionView::onDistributionsUpdated(const std::vector<boost::math::beta_distribution<>> distributions)
{
  chart_->removeAllSeries();

  std::vector<double> x_target = MathUtils::linespace(0.0, 1.0, 100);
  for (const boost::math::beta_distribution<>& dist : distributions)
  {
    QLineSeries* lineSeries = new QLineSeries();

    for (double x : x_target)
    {
      *lineSeries << QPointF(x, pdf(dist, x));
    }

    chart_->addSeries(lineSeries);
  }
}

void DistributionView::setupChartAndChartView()
{
  chart_->legend()->hide();
  chart_->createDefaultAxes();
  //  chart_->axisY()->setRange(0, 20);
  chart_->setTitle("Parameter 0");
  chart_->show();

  chartView_->setRenderHint(QPainter::Antialiasing);
}