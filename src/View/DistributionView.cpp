#include "View/DistributionView.h"
#include "MathUtils.h"

DistributionView::DistributionView(QWidget* parent)
  : QWidget(parent)
  , chart_(new QChart())
  , axisX_(new QValueAxis(chart_))
  , axisY_(new QValueAxis(chart_))
  , chartView_(new QChartView(chart_, this))
  , window_(new QMainWindow(this))
{
  setupChartAndChartView();
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
    lineSeries->attachAxis(axisX_);
    lineSeries->attachAxis(axisY_);
  }
}

void DistributionView::setupChartAndChartView()
{
  axisX_->setRange(0.0, 1.0);
  chart_->addAxis(axisX_, Qt::AlignBottom);

  axisY_->setRange(0.0, 20.0);
  chart_->addAxis(axisY_, Qt::AlignLeft);

  chart_->legend()->hide();
  chart_->setTitle("Distributions");

  chartView_->setRenderHint(QPainter::Antialiasing);

  window_->setCentralWidget(chartView_);
  window_->resize(400, 300);
  window_->show();
}