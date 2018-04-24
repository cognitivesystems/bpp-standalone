#include "ViewMgr/DistributionViewMgr.h"

DistributionViewMgr::DistributionViewMgr(QObject* parent, filter::ParticleFilter& particleFilter,
                                         DistributionView& distributionView)
  : QObject(parent), particleFilter_(particleFilter), distributionView_(distributionView)
{
  wireParticleFilterAndDistributionView();
}

void DistributionViewMgr::wireParticleFilterAndDistributionView()
{
  connect(&particleFilter_, &filter::ParticleFilter::notifyDistributionsUpdated, &distributionView_,
          &DistributionView::onDistributionsUpdated);
}