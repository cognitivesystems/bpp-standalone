#include <Qt3DRender/QTextureImage>
#include "View/BoxEntity.h"

BoxEntity::BoxEntity(Qt3DCore::QNode* parent)
  : Qt3DCore::QEntity(parent), mesh_(new Qt3DRender::QMesh()), transform_(new Qt3DCore::QTransform())
{
  addComponent(mesh_);
  addComponent(transform_);
}

Qt3DRender::QMesh* BoxEntity::mesh() const
{
  return mesh_;
}

Qt3DCore::QTransform* BoxEntity::transform() const
{
  return transform_;
}

void BoxEntity::setTexture(const QUrl& textureImage)
{
  Qt3DExtras::QNormalDiffuseSpecularMapMaterial* normalDiffuseSpecularMapMaterial =
      new Qt3DExtras::QNormalDiffuseSpecularMapMaterial();
  Qt3DRender::QTextureImage* diffuseImage = new Qt3DRender::QTextureImage();
  diffuseImage->setSource(textureImage);
  normalDiffuseSpecularMapMaterial->diffuse()->addTextureImage(diffuseImage);

  Qt3DRender::QTextureImage* specularImage = new Qt3DRender::QTextureImage();
  specularImage->setSource(textureImage);
  normalDiffuseSpecularMapMaterial->specular()->addTextureImage(specularImage);

  Qt3DRender::QTextureImage* normalImage = new Qt3DRender::QTextureImage();
  normalImage->setSource(textureImage);
  normalDiffuseSpecularMapMaterial->normal()->addTextureImage(normalImage);

  normalDiffuseSpecularMapMaterial->setShininess(1.0);
  normalDiffuseSpecularMapMaterial->setAmbient("white");

  addComponent(normalDiffuseSpecularMapMaterial);
}
