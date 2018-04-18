#include "BinPackingViewMgr.h"

BinPackingViewMgr::BinPackingViewMgr(QObject* parent, BoxSet& boxSet, BinPackingView& binPackingView)
  : QObject(parent), boxSet_(boxSet), binPackingView_(binPackingView)
{
  wireBoxSetAndBinPackingView();
}

void BinPackingViewMgr::wireBoxSetAndBinPackingView()
{
  connect(&boxSet_, &BoxSet::notifyBoxesAdded, &binPackingView_, &BinPackingView::onBoxesAdded);
  connect(&boxSet_, &BoxSet::notifyBoxesUpdated, &binPackingView_, &BinPackingView::onBoxesUpdated);
}