#ifndef OBJENTITY_H
#define OBJENTITY_H

#include <QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QMesh>
#include <Qt3DExtras/QNormalDiffuseSpecularMapMaterial>

class ObjEntity : public Qt3DCore::QEntity
{
public:
  explicit ObjEntity(Qt3DCore::QNode* parent = nullptr);

  Qt3DRender::QMesh* mesh() const;
  Qt3DCore::QTransform* transform() const;
  Qt3DExtras::QNormalDiffuseSpecularMapMaterial* material() const;

private:
  Qt3DRender::QMesh* mesh_;
  Qt3DCore::QTransform* transform_;
  Qt3DExtras::QNormalDiffuseSpecularMapMaterial* material_;
};

#endif  // OBJENTITY_H
