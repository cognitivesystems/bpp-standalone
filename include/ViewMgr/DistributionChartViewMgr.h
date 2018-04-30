#ifndef DISTRIBUTIONCHARTVIEWMGR_H
#define DISTRIBUTIONCHARTVIEWMGR_H

#include "opt/ParticleFilter.h"
#include "View/DistributionChart.h"
#include <QObject>

class DistributionChartViewMgr : public QObject
{
public:
  explicit DistributionChartViewMgr(QObject* parent, filter::ParticleFilter& particleFilter,
                                    DistributionChart& distributionChart);

private:
  filter::ParticleFilter& particleFilter_;
  DistributionChart& distributionChart_;

  void wireParticleFilterAndDistributionView();
};

#endif  // DISTRIBUTIONCHARTVIEWMGR_H