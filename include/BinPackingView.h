#ifndef BINPACKINGVIEW_H
#define BINPACKINGVIEW_H

#include <QSharedPointer>
#include <QString>
#include <Qt3DCore/QAspectEngine>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <QWidget>
#include <Qt3DCore/QComponent>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QMesh>
#include <Qt3DRender/qrenderaspect.h>

#include <iostream>
#include "objentity.h"
#include "boxentity.h"
#include "Box.h"

class BinPackingView : public QWidget
{
  Q_OBJECT

public:
  explicit BinPackingView(QWidget* parent = 0);

  Qt3DCore::QEntity* getScene();
  void clearScene();

  void addObjEntity(const std::string obj_url);

  void addBoxEntity(const bpa::Box& box);
  void updateBoxEntity(const bpa::Box& box);
  void removeBoxEntity(const bpa::Box& box);
  void removeAllBoxEntities();

public slots:
  void onBoxAdded(const bpa::Box& box);
  void onBoxUpdated(const bpa::Box& box);
  void onBoxRemoved(const bpa::Box& box);

  void onBoxesAdded(const std::vector<bpa::Box>& boxes);
  void onBoxesUpdated(const std::vector<bpa::Box>& boxes);
  void onBoxesRemoved(const std::vector<bpa::Box>& boxes);

private:
  Qt3DCore::QEntity* root_;
  QMap<QString, BoxEntity*> uuid_entity_map_;
};

#endif  // BINPACKINGVIEW_H