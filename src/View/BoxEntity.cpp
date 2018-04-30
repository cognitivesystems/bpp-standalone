#include "View/BoxEntity.h"

#include <Qt3DRender/QTextureImage>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QMesh>
#include <Qt3DExtras/QNormalDiffuseSpecularMapMaterial>

BoxEntity::BoxEntity(Qt3DCore::QNode* parent) : Qt3DCore::QEntity(parent), transform_(new Qt3DCore::QTransform())
{
  addComponent(transform_);
}

Qt3DCore::QTransform* BoxEntity::transform() const
{
  return transform_;
}

void BoxEntity::setMesh(const QUrl& meshSource)
{
  Qt3DRender::QMesh* mesh = new Qt3DRender::QMesh();
  mesh->setSource(meshSource);
  addComponent(mesh);
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

void BoxEntity::generateMeshAndTexture(const QVector3D& vector3D)
{
  Qt3DExtras::QCuboidMesh* cuboidMesh = new Qt3DExtras::QCuboidMesh();
  cuboidMesh->setXExtent(vector3D[0]);
  cuboidMesh->setYExtent(vector3D[1]);
  cuboidMesh->setZExtent(vector3D[2]);
  addComponent(cuboidMesh);

  Qt3DExtras::QPhongMaterial* phongMaterial = new Qt3DExtras::QPhongMaterial();
  phongMaterial->setDiffuse(QColor(qrand() % 255, qrand() % 255, qrand() % 255, 1));
  addComponent(phongMaterial);
}