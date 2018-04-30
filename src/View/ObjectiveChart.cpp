#include "View/ObjectiveChart.h"

ObjectiveChart::ObjectiveChart(QWidget* parent)
  : QWidget(parent)
  , chart_(new QChart())
  , axisX_(new QValueAxis(chart_))
  , axisY_(new QValueAxis(chart_))
  , chartView_(new QChartView(chart_, this))
  , ui_(new Ui::ObjectiveChart)
{
  ui_->setupUi(this);
  setupChartAndChartView();
}

ObjectiveChart::~ObjectiveChart()
{
  delete ui_;
}

void ObjectiveChart::onObjectivesUpdated(const std::map<std::string, std::vector<double>>& objectivesMap)
{
  chart_->removeAllSeries();

  for (const auto& objectives : objectivesMap)
  {
    QLineSeries* lineSeries = new QLineSeries();
    lineSeries->setName(QString::fromStdString(objectives.first));

    for (size_t i = 0; i != objectives.second.size(); ++i)
    {
      *lineSeries << QPointF(i + 1, objectives.second[i]);
    }

    chart_->addSeries(lineSeries);
    lineSeries->attachAxis(axisX_);
    lineSeries->attachAxis(axisY_);
  }
}

void ObjectiveChart::onAllObjectivesRemoved()
{
  chart_->removeAllSeries();
}

void ObjectiveChart::setupChartAndChartView()
{
  axisX_->setRange(0, 10);
  chart_->addAxis(axisX_, Qt::AlignBottom);

  axisY_->setRange(0.0, 1.0);
  chart_->addAxis(axisY_, Qt::AlignLeft);

  chart_->legend()->setAlignment(Qt::AlignRight);
  chart_->setTitle("Objectives");

  chartView_->setRenderHint(QPainter::Antialiasing);
  ui_->lohorizontal->addWidget(chartView_);
}