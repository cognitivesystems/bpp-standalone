#ifndef OBJECTIVES_H
#define OBJECTIVES_H

#include <QObject>
#include <map>
#include <string>
#include <vector>

class Objectives : public QObject
{
  Q_OBJECT

public:
  explicit Objectives(QObject* parent = nullptr);

  void addObjectiveValue(const std::string& name, double value);
  void removeAllObjectives();

signals:
  void notifyObjectivesUpdated(const std::map<std::string, std::vector<double>>& objectivesMap);
  void notifyAllObjectivesRemoved();

private:
  std::map<std::string, std::vector<double>> objectivesMap_;
};

#endif  // OBJECTIVES_H