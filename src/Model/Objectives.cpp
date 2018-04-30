#include "Model/Objectives.h"

Objectives::Objectives(QObject* parent) : QObject(parent)
{
}

void Objectives::addObjectiveValue(const std::string& name, double value)
{
  objectivesMap_[name].push_back(value);
  emit notifyObjectivesUpdated(objectivesMap_);
}

void Objectives::removeAllObjectives()
{
  objectivesMap_.clear();
  emit notifyAllObjectivesRemoved();
}