#include "objentity.h"

ObjEntity::ObjEntity(Qt3DCore::QNode* parent)
  : Qt3DCore::QEntity(parent)
  , mesh_(new Qt3DRender::QMesh())
  , transform_(new Qt3DCore::QTransform())
  , material_(new Qt3DExtras::QNormalDiffuseSpecularMapMaterial())
{
  addComponent(mesh_);
  addComponent(transform_);
  addComponent(material_);
}

Qt3DRender::QMesh* ObjEntity::mesh() const
{
  return mesh_;
}

Qt3DCore::QTransform* ObjEntity::transform() const
{
  return transform_;
}

Qt3DExtras::QNormalDiffuseSpecularMapMaterial* ObjEntity::material() const
{
  return material_;
}
