#ifndef BOXENTITY_H
#define BOXENTITY_H

#include <QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QMesh>
#include <Qt3DExtras/QNormalDiffuseSpecularMapMaterial>

class BoxEntity : public Qt3DCore::QEntity
{
public:
  explicit BoxEntity(Qt3DCore::QNode* parent = nullptr);

  Qt3DRender::QMesh* mesh() const;
  Qt3DCore::QTransform* transform() const;
  void setTexture(const QUrl& textureImage);

private:
  Qt3DRender::QMesh* mesh_;
  Qt3DCore::QTransform* transform_;
};

#endif  // BOXENTITY_H
