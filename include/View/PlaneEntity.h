#ifndef PLANEENTITY_H
#define PLANEENTITY_H

#include <Qt3DCore/QEntity>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/QPhongMaterial>

class PlaneEntity : public Qt3DCore::QEntity
{
public:
  explicit PlaneEntity(Qt3DCore::QNode* parent = 0);

  Qt3DExtras::QPlaneMesh* mesh() const;
  Qt3DCore::QTransform* transform() const;
  Qt3DExtras::QPhongMaterial* material() const;

private:
  Qt3DExtras::QPlaneMesh* mesh_;
  Qt3DCore::QTransform* transform_;
  Qt3DExtras::QPhongMaterial* material_;
};

#endif  // PLANEENTITY_H