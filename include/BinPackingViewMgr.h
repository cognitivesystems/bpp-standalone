#ifndef BINPACKINGVIEWMGR_H
#define BINPACKINGVIEWMGR_H

#include "BoxSet.h"
#include "BinPackingView.h"
#include <QObject>

class BinPackingViewMgr : public QObject
{
public:
  explicit BinPackingViewMgr(QObject* parent, BoxSet& boxSet, BinPackingView& binPackingView);

private:
  BoxSet& boxSet_;
  BinPackingView& binPackingView_;

  void wireBoxSetAndBinPackingView();
};

#endif  // BINPACKINGVIEWMGR_H