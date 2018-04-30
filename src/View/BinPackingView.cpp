#include "View/BinPackingView.h"
#include <QTextureImage>
#include <QtCore/QCoreApplication>
#include <QtCore/QFileInfo>

BinPackingView::BinPackingView(QWidget* parent)
  : QWidget(parent), root_(new Qt3DCore::QEntity()), floor_(new PlaneEntity(root_)), pallet_(new PlaneEntity(root_))
{
  setupFloorAndPallet();
}

Qt3DCore::QEntity* BinPackingView::getScene()
{
  return root_;
}

void BinPackingView::clearScene()
{
  removeAllBoxEntities();
}

void BinPackingView::addBoxEntity(const bpa::Box& box)
{
  BoxEntity* boxEntityPtr = new BoxEntity(root_);

  boxEntityPtr->transform()->setTranslation(
      QVector3D(box.position.position[0], box.position.position[1], box.position.position[2]));

  double box_rot = box.is_rotated ? 90.0 : 0.0;
  boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, box_rot));

  QString resourceDir = QCoreApplication::applicationDirPath() + "/../src/resources";
  QString objFile = resourceDir + "/obj/" + QString::fromStdString(box.m_name) + ".obj";
  QString textureImage = resourceDir + "/textures/" + QString::fromStdString(box.m_name) + ".png";
  if (QFileInfo::exists(objFile) && QFileInfo::exists(textureImage))
  {
    boxEntityPtr->setMesh(QUrl::fromLocalFile(objFile));
    boxEntityPtr->setTexture(QUrl::fromLocalFile(textureImage));
  }
  else
  {
    boxEntityPtr->generateMeshAndTexture(QVector3D(box.m_length, box.m_width, box.m_height));
  }

  if (box.m_length == 0.3 || box.m_width == 0.3)
  {
    qDebug() << "Model box --> " << boxEntityPtr->transform()->translation();
  }

  std::cout << "Name --> " << box.m_name << std::endl;
  qDebug() << boxEntityPtr->transform();
  qDebug() << boxEntityPtr->transform()->translation();
  qDebug() << boxEntityPtr->transform()->rotation();

  uuid_entity_map_[box.m_name.c_str()] = boxEntityPtr;
}

void BinPackingView::updateBoxEntity(const bpa::Box& box)
{
  BoxEntity* boxEntityPtr = uuid_entity_map_[box.m_name.c_str()];

  boxEntityPtr->transform()->setTranslation(QVector3D(0, 0, 0));

  double box_rot = box.is_rotated ? 90.0 : 0.0;
  boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, box_rot));

  boxEntityPtr->transform()->setTranslation(
      QVector3D(box.position.position[0], box.position.position[1], box.position.position[2]));
}

void BinPackingView::removeBoxEntity(const bpa::Box& box)
{
  QMap<QString, BoxEntity*>::iterator itr = uuid_entity_map_.find(box.m_name.c_str());
  if (itr != uuid_entity_map_.end())
  {
    uuid_entity_map_.erase(itr);
  }
}

void BinPackingView::removeAllBoxEntities()
{
  if (!uuid_entity_map_.empty())
  {
    qDeleteAll(uuid_entity_map_);
    uuid_entity_map_.clear();
  }
}

void BinPackingView::onBoxesLoaded(const std::vector<bpa::Box>& boxes)
{
  for (const bpa::Box& box : boxes)
  {
    addBoxEntity(box);
  }
}

void BinPackingView::onBoxesUpdated(const std::vector<bpa::Box>& boxes)
{
  for (const bpa::Box& box : boxes)
  {
    updateBoxEntity(box);
  }
}

void BinPackingView::onBoxesRemoved(const std::vector<bpa::Box>& boxes)
{
  for (const bpa::Box& box : boxes)
  {
    removeBoxEntity(box);
  }
}

void BinPackingView::onAllBoxesRemoved()
{
  removeAllBoxEntities();
}

void BinPackingView::setupFloorAndPallet()
{
  // values are incorrect, update needed

  floor_->mesh()->setHeight(15.0);
  floor_->mesh()->setWidth(15.0);
  floor_->transform()->setTranslation(QVector3D(0.0, 0.0, 0.0));
  floor_->transform()->setRotation(QQuaternion::fromEulerAngles(90.0, 0.0, 0.0));
  floor_->material()->setDiffuse(QColor(255, 0, 0, 1));

  pallet_->mesh()->setHeight(2.28);
  pallet_->mesh()->setWidth(3.00);
  pallet_->transform()->setTranslation(QVector3D(1.14, 1.5, 0.0));
  pallet_->transform()->setRotation(QQuaternion::fromEulerAngles(90.0, 0.0, 0.0));
  pallet_->material()->setDiffuse(QColor(0, 0, 255, 1));
}