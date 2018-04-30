#ifndef BOXENTITY_H
#define BOXENTITY_H

#include <QEntity>
#include <Qt3DCore/QTransform>

class BoxEntity : public Qt3DCore::QEntity
{
public:
  explicit BoxEntity(Qt3DCore::QNode* parent = nullptr);

  Qt3DCore::QTransform* transform() const;
  void setMesh(const QUrl& meshSource);
  void setTexture(const QUrl& textureImage);
  void generateMeshAndTexture(const QVector3D& vector3D);

private:
  Qt3DCore::QTransform* transform_;
};

#endif  // BOXENTITY_H
