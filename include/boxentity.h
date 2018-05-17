#ifndef BOXENTITY_H
#define BOXENTITY_H

#include <QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongAlphaMaterial>

class BoxEntity : public Qt3DCore::QEntity
{
public:
  explicit BoxEntity(Qt3DCore::QNode* parent = nullptr);

  Qt3DExtras::QCuboidMesh* mesh() const;
  Qt3DCore::QTransform* transform() const;
  Qt3DExtras::QPhongAlphaMaterial* material() const;

private:
  Qt3DExtras::QCuboidMesh* mesh_;
  Qt3DCore::QTransform* transform_;
  Qt3DExtras::QPhongAlphaMaterial* material_;
};

#endif  // BOXENTITY_H
