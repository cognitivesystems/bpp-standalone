#include "ViewMgr/DistributionChartViewMgr.h"

DistributionChartViewMgr::DistributionChartViewMgr(QObject* parent, filter::ParticleFilter& particleFilter,
                                                   DistributionChart& distributionChart)
  : QObject(parent), particleFilter_(particleFilter), distributionChart_(distributionChart)
{
  wireParticleFilterAndDistributionView();
}

void DistributionChartViewMgr::wireParticleFilterAndDistributionView()
{
  connect(&particleFilter_, &filter::ParticleFilter::notifyDistributionsUpdated, &distributionChart_,
          &DistributionChart::onDistributionsUpdated);
}