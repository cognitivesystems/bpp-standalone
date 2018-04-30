#include "ViewMgr/ObjectiveChartViewMgr.h"

ObjectiveChartViewMgr::ObjectiveChartViewMgr(QObject* parent, Objectives& objectives, ObjectiveChart& objectiveChart)
  : QObject(parent), objectives_(objectives), objectiveChart_(objectiveChart)
{
  wireObjectivesAndObjectiveChart();
}

void ObjectiveChartViewMgr::wireObjectivesAndObjectiveChart()
{
  connect(&objectives_, &Objectives::notifyObjectivesUpdated, &objectiveChart_, &ObjectiveChart::onObjectivesUpdated);
  connect(&objectives_, &Objectives::notifyAllObjectivesRemoved, &objectiveChart_,
          &ObjectiveChart::onAllObjectivesRemoved);
}