#ifndef BOXSET_H
#define BOXSET_H

#include "Box.h"
#include "BoxJsonParser.h"
#include <vector>
#include <QObject>

class BoxSet : public QObject
{
  Q_OBJECT

public:
  explicit BoxSet(QObject* parent = nullptr);

  void loadBoxesFromFile(const QString& fileName);
  void updateBoxes(const std::vector<bpa::Box>& boxes);
  void removeAllBoxes();

  std::vector<bpa::Box> getBoxes();

signals:
  void notifyBoxesLoaded(const std::vector<bpa::Box>& boxes);
  void notifyBoxesUpdated(const std::vector<bpa::Box>& boxes);
  void notifyAllBoxesRemoved();

private:
  std::vector<bpa::Box> boxes_;
};

#endif  // BOXSET_H
