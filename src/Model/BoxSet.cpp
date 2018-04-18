#include "Model/BoxSet.h"

BoxSet::BoxSet(QObject* parent) : QObject(parent)
{
}

void BoxSet::addBoxesFromFile(const QString& fileName)
{
  boxes_ = box_factory::BoxJsonParser::getBoxesFromJsonFile(fileName);
  emit notifyBoxesAdded(boxes_);
}

void BoxSet::updateBoxes(const std::vector<bpa::Box> &boxes)
{
  boxes_ = boxes;
  emit notifyBoxesUpdated(boxes_);
}

void BoxSet::removeAllBoxes()
{
  if (!boxes_.empty())
  {
    boxes_.clear();
  }
  emit notifyAllBoxesRemoved();
}

std::vector<bpa::Box> BoxSet::getBoxes()
{
  return boxes_;
}