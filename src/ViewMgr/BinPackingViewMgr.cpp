#include "ViewMgr/BinPackingViewMgr.h"

BinPackingViewMgr::BinPackingViewMgr(QObject* parent, BoxSet& boxSet, BinPackingView& binPackingView)
  : QObject(parent), boxSet_(boxSet), binPackingView_(binPackingView)
{
  wireBoxSetAndBinPackingView();
}

void BinPackingViewMgr::wireBoxSetAndBinPackingView()
{
  connect(&boxSet_, &BoxSet::notifyBoxesLoaded, &binPackingView_, &BinPackingView::onBoxesLoaded);
  connect(&boxSet_, &BoxSet::notifyBoxesUpdated, &binPackingView_, &BinPackingView::onBoxesUpdated);
  connect(&boxSet_, &BoxSet::notifyAllBoxesRemoved, &binPackingView_, &BinPackingView::onAllBoxesRemoved);
}