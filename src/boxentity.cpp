#include "boxentity.h"

BoxEntity::BoxEntity(Qt3DCore::QNode* parent)
  : Qt3DCore::QEntity(parent)
  , mesh_(new Qt3DExtras::QCuboidMesh())
  , transform_(new Qt3DCore::QTransform())
  , material_(new Qt3DExtras::QPhongAlphaMaterial())
{
  addComponent(mesh_);
  addComponent(transform_);
  addComponent(material_);
}

Qt3DExtras::QCuboidMesh* BoxEntity::mesh() const
{
  return mesh_;
}

Qt3DCore::QTransform* BoxEntity::transform() const
{
  return transform_;
}

Qt3DExtras::QPhongAlphaMaterial *BoxEntity::material() const
{
  return material_;
}
