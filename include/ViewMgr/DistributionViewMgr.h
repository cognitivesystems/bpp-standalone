#ifndef DISTRIBUTIONVIEWMGR_H
#define DISTRIBUTIONVIEWMGR_H

#include "opt/ParticleFilter.h"
#include "View/DistributionView.h"
#include <QObject>

class DistributionViewMgr : public QObject
{
public:
  explicit DistributionViewMgr(QObject* parent, filter::ParticleFilter& particleFilter,
                               DistributionView& distributionView);

private:
  filter::ParticleFilter& particleFilter_;
  DistributionView& distributionView_;

  void wireParticleFilterAndDistributionView();
};

#endif  // DISTRIBUTIONVIEWMGR_H