#include "scenerenderer3d.h"

SceneRenderer3D::SceneRenderer3D(QWidget* parent) : QWidget(parent)
{
  root_ = new Qt3DCore::QEntity;
}

void SceneRenderer3D::addObjectBody(const ObjectBody& info)
{
  if (info.is_box)
  {
    Qt3DCore::QEntity* box = new Qt3DCore::QEntity(root_);

    //        Qt3DRender::QMesh* mesh  = new Qt3DRender::QMesh(root_);
    //        mesh->setSource(QUrl::fromLocalFile("0b74db5d.obj"));

    Qt3DExtras::QCuboidMesh* cuboid = new Qt3DExtras::QCuboidMesh();
    cuboid->setXExtent(info.bbox.x());
    cuboid->setYExtent(info.bbox.y());
    cuboid->setZExtent(info.bbox.z());

    // CuboidMesh Transform
    Qt3DCore::QTransform* cuboidTransform = new Qt3DCore::QTransform();
    cuboidTransform->setTranslation(info.pose.position);
    //        cuboidTransform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),
    //        45.f ));
    cuboidTransform->setRotation(info.pose.orientation);

    Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial(root_);

    material->setDiffuse(QColor(qrand() % 255, qrand() % 255, qrand() % 255));

    box->addComponent(cuboid);
    box->addComponent(cuboidTransform);
    box->addComponent(material);

    model_entity_map_[info.body_name] = box;
    body_transform_map_[info.body_name] = cuboidTransform;
  }
}

void SceneRenderer3D::addObjectModel(const ObjectModel& info)
{
  for (auto b : info.bodies)
  {
    addObjectBody(b);
  }
}

void SceneRenderer3D::removeObjectModel(const ObjectModel& model)
{
  std::cout << "removeObjectModel" << std::endl;
  for (ObjectBody b : model.bodies)
  {
    Qt3DCore::QEntity* entity = model_entity_map_[b.body_name];
    foreach (Qt3DCore::QComponent* component, entity->components())
    {
      entity->removeComponent(component);
      delete (component);
      component = NULL;
    }
  }
}

void SceneRenderer3D::updateObjectModel(const ObjectModel& info)
{
  for (ObjectBody b : info.bodies)
  {
    Qt3DCore::QTransform* tr = body_transform_map_[b.body_name];
    tr->setTranslation(b.pose.position);
    tr->setRotation(b.pose.orientation);
  }
}

ObjectModels SceneRenderer3D::getModels()
{
  return current_scene_models_;
}

void SceneRenderer3D::deleteScene()
{
}

void SceneRenderer3D::clearScene()
{
}

void SceneRenderer3D::createTestScene()
{
}

Qt3DCore::QEntity* SceneRenderer3D::getScene()
{
  return root_;
}

void SceneRenderer3D::slotAddObjectModel(const ObjectModel& model)
{
  addObjectModel(model);
}

void SceneRenderer3D::slotRemoveObjectModel(const ObjectModel& model)
{
  removeObjectModel(model);
}

void SceneRenderer3D::slotUpdateObjectModel(const ObjectModel& model)
{
  updateObjectModel(model);
}

void SceneRenderer3D::slotUpdateObjectModels(const std::vector<ObjectModel>& models)
{
  for (auto model : models)
  {
    addObjectModel(model);
  }
}
