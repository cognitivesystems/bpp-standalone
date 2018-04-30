#ifndef DASHBOARDVIEW_H
#define DASHBOARDVIEW_H

#include "DistributionChart.h"
#include "ObjectiveChart.h"
#include "ui_DashboardView.h"
#include <QMainWindow>

namespace Ui
{
class DashboardView;
}

class DashboardView : public QMainWindow
{
  Q_OBJECT

public:
  explicit DashboardView(QWidget* parent, DistributionChart& distributionChart, ObjectiveChart& objectiveChart);
  ~DashboardView();

private:
  DistributionChart& distributionChart_;
  ObjectiveChart& objectiveChart_;
  Ui::DashboardView* ui_;
};

#endif  // DASHBOARDVIEW_H
