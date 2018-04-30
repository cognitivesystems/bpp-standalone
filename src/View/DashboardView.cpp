#include "View/DashboardView.h"

DashboardView::DashboardView(QWidget* parent, DistributionChart& distributionChart)
  : QMainWindow(parent), distributionChart_(distributionChart), ui_(new Ui::DashboardView)
{
  ui_->setupUi(this);

  distributionChart_.setParent(this);
  ui_->loDistributionChart->addWidget(&distributionChart_);

  this->show();
}

DashboardView::~DashboardView()
{
  delete ui_;
}
