#include "View/DashboardView.h"

DashboardView::DashboardView(QWidget* parent, DistributionChart& distributionChart, ObjectiveChart& objectiveChart)
  : QMainWindow(parent)
  , distributionChart_(distributionChart)
  , objectiveChart_(objectiveChart)
  , ui_(new Ui::DashboardView)
{
  ui_->setupUi(this);

  distributionChart_.setParent(this);
  ui_->loDistributionChart->addWidget(&distributionChart_);

  objectiveChart_.setParent(this);
  ui_->loObjectiveChart->addWidget(&objectiveChart_);

  this->show();
}

DashboardView::~DashboardView()
{
  delete ui_;
}
