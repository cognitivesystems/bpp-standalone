#include "scenerenderer3d.h"

SceneRenderer3D::SceneRenderer3D(QWidget* parent) : QWidget(parent), root_(new Qt3DCore::QEntity())
{
}

void SceneRenderer3D::addBoxEntity(const bpa::Box& box)
{
  BoxEntity* boxEntityPtr = new BoxEntity(root_);

  boxEntityPtr->mesh()->setXExtent(box.m_length);
  boxEntityPtr->mesh()->setYExtent(box.m_width);
  boxEntityPtr->mesh()->setZExtent(box.m_height);

  boxEntityPtr->transform()->setTranslation(
      QVector3D(box.position.position[0], box.position.position[1], box.position.position[2]));

  double box_rot = box.is_rotated ? M_PI_2 : 0.0;
  boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, box_rot, 0));

  boxEntityPtr->material()->setDiffuse(QColor(qrand() % 255, qrand() % 255, qrand() % 255));

  uuid_entity_map_[box.m_name.c_str()] = boxEntityPtr;
}

void SceneRenderer3D::updateBoxEntity(const bpa::Box& box)
{
  BoxEntity* boxEntityPtr = uuid_entity_map_[box.m_name.c_str()];

  boxEntityPtr->transform()->setTranslation(
      QVector3D(box.position.position[0], box.position.position[1], box.position.position[2]));

  double box_rot = box.is_rotated ? M_PI_2 : 0.0;
  boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, box_rot, 0));
}

void SceneRenderer3D::removeBoxEntity(const bpa::Box& box)
{
  QMap<QString, BoxEntity*>::iterator itr = uuid_entity_map_.find(box.m_name.c_str());
  if (itr != uuid_entity_map_.end())
  {
    uuid_entity_map_.erase(itr);
  }
}

void SceneRenderer3D::removeAllBoxEntities()
{
  if (!uuid_entity_map_.empty())
  {
    qDeleteAll(uuid_entity_map_);
    uuid_entity_map_.clear();
  }
}

void SceneRenderer3D::deleteScene()
{
}

void SceneRenderer3D::clearScene()
{
  removeAllBoxEntities();
}

void SceneRenderer3D::createTestScene()
{
}

Qt3DCore::QEntity* SceneRenderer3D::getScene()
{
  return root_;
}

void SceneRenderer3D::slotAddBoxEntity(const bpa::Box& box)
{
  addBoxEntity(box);
}

void SceneRenderer3D::slotUpdateBoxEntity(const bpa::Box& box)
{
  updateBoxEntity(box);
}

void SceneRenderer3D::slotRemoveBoxEntity(const bpa::Box& box)
{
  removeBoxEntity(box);
}

void SceneRenderer3D::slotUpdateBoxEntities(const std::vector<bpa::Box>& boxes)
{
  for (const bpa::Box& box : boxes)
  {
    removeBoxEntity(box);
  }
}