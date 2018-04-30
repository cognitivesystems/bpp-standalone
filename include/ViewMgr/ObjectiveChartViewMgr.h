#ifndef OBJECTIVECHARTVIEWMGR_H
#define OBJECTIVECHARTVIEWMGR_H

#include "Model/Objectives.h"
#include "View/ObjectiveChart.h"
#include <QObject>

class ObjectiveChartViewMgr : public QObject
{
  Q_OBJECT

public:
  explicit ObjectiveChartViewMgr(QObject* parent, Objectives& objectives, ObjectiveChart& objectiveChart);

private:
  Objectives& objectives_;
  ObjectiveChart& objectiveChart_;

  void wireObjectivesAndObjectiveChart();
};

#endif  // OBJECTIVECHARTVIEWMGR_H