#include "View/PlaneEntity.h"

PlaneEntity::PlaneEntity(Qt3DCore::QNode* parent)
  : Qt3DCore::QEntity(new Qt3DCore::QEntity(parent))
  , mesh_(new Qt3DExtras::QPlaneMesh())
  , transform_(new Qt3DCore::QTransform())
  , material_(new Qt3DExtras::QPhongMaterial())
{
  addComponent(mesh_);
  addComponent(transform_);
  addComponent(material_);
}

Qt3DExtras::QPlaneMesh* PlaneEntity::mesh() const
{
  return mesh_;
}

Qt3DCore::QTransform* PlaneEntity::transform() const
{
  return transform_;
}

Qt3DExtras::QPhongMaterial* PlaneEntity::material() const
{
  return material_;
}