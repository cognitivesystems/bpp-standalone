#include "boxentity.h"

BoxEntity::BoxEntity(QString name, Qt3DCore::QNode* parent)
  : Qt3DCore::QEntity(parent)
  , mesh_(new Qt3DExtras::QCuboidMesh())
  , transform_(new Qt3DCore::QTransform())
  , material_(new Qt3DExtras::QPhongMaterial())
  , picker_(new Qt3DRender::QObjectPicker(parent))
  , base_name_(name)
{
  setObjectName(base_name_);
  mesh_->setObjectName(base_name_+QString("_mesh"));
  transform_->setObjectName(base_name_+QString("_transform"));
  material_->setObjectName(base_name_+QString("_material"));

  picker_->setHoverEnabled(false);
  picker_->setEnabled(true);
  picker_->setObjectName(base_name_+QString("_picker"));

  qDebug() << "++++++++++++++++++++++++++++++++++ " << objectName();

  addComponent(mesh_);
  addComponent(transform_);
  addComponent(material_);
  addComponent(picker_);
}

Qt3DExtras::QCuboidMesh* BoxEntity::mesh() const
{
  return mesh_;
}

Qt3DCore::QTransform* BoxEntity::transform() const
{
  return transform_;
}

Qt3DExtras::QPhongMaterial *BoxEntity::material() const
{
    return material_;
}

Qt3DRender::QObjectPicker *BoxEntity::picker() const
{
    return picker_;
}
