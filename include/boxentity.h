#ifndef BOXENTITY_H
#define BOXENTITY_H

#include <QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QObjectPicker>

class BoxEntity : public Qt3DCore::QEntity
{
public:
  explicit BoxEntity(QString name, Qt3DCore::QNode* parent = nullptr);

  Qt3DExtras::QCuboidMesh* mesh() const;
  Qt3DCore::QTransform* transform() const;
  Qt3DExtras::QPhongMaterial* material() const;
  Qt3DRender::QObjectPicker* picker() const;

private:
  Qt3DExtras::QCuboidMesh* mesh_;
  Qt3DCore::QTransform* transform_;
  Qt3DExtras::QPhongMaterial* material_;
  Qt3DRender::QObjectPicker *picker_;
  QString base_name_;
};

#endif  // BOXENTITY_H
