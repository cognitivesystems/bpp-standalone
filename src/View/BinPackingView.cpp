#include "View/BinPackingView.h"
#include <QTextureImage>

BinPackingView::BinPackingView(QWidget* parent) : QWidget(parent), root_(new Qt3DCore::QEntity())
{
}

void BinPackingView::addObjEntity(const string obj_url)
{
  ObjEntity* objEntityPtr = new ObjEntity(root_);
  objEntityPtr->mesh()->setSource(QUrl::fromLocalFile(obj_url.c_str()));
  objEntityPtr->transform()->setTranslation(QVector3D(0, 0, 0));
  objEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, 0));

//  Qt3DRender::QTextureImage* textImg1 = new Qt3DRender::QTextureImage();
//  std::string name = "/home/nair/workspace/bpp_code/bpp-standalone/build/mesh_material0000_map_Kd.png";
//  textImg1->setSource(QUrl::fromLocalFile(name.c_str()));

//  objEntityPtr->material()->diffuse()->addTextureImage(textImg1);
//  objEntityPtr->material()->normal()->addTextureImage(textImg1);
//  objEntityPtr->material()->specular()->addTextureImage(textImg1);
  objEntityPtr->material()->setShininess(1.0);
  objEntityPtr->material()->setAmbient("white");
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

  boxEntityPtr->mesh()->setXExtent(box.m_length);
  boxEntityPtr->mesh()->setYExtent(box.m_width);
  boxEntityPtr->mesh()->setZExtent(box.m_height);

  boxEntityPtr->transform()->setTranslation(QVector3D(0, 0, 0));

  double box_rot = box.is_rotated ? 90.0 : 0.0;

  QColor color = QColor(qrand() % 255, qrand() % 255, qrand() % 255);

  if (box.m_type == "box")
  {
    boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, box_rot));
    boxEntityPtr->material()->setDiffuse(QColor(qrand() % 255, qrand() % 255, qrand() % 255, 1));
  }
  else if (box.m_type == "pallet")
  {
    boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, 0));
    boxEntityPtr->material()->setDiffuse(QColor(255, 0, 0, 1));
  }
  else
  {
    boxEntityPtr->transform()->setRotation(QQuaternion::fromEulerAngles(0, 0, 0));
    boxEntityPtr->material()->setDiffuse(QColor(0, 0, 255, 1));
  }

  boxEntityPtr->transform()->setTranslation(
      QVector3D(box.position.position[0], box.position.position[1], box.position.position[2]));

  if (box.m_length == 0.3 || box.m_width == 0.3)
  {
    qDebug() << "Model box --> " << boxEntityPtr->transform()->translation();
  }

  std::cout << "Name --> " << box.m_name << std::endl;
  qDebug() << boxEntityPtr->transform();
  qDebug() << boxEntityPtr->transform()->translation();
  qDebug() << boxEntityPtr->transform()->rotation();

  //      boxEntityPtr->material()->setShininess(0.0);

  //  if(box.m_name=="pallet")
  //  {
  //      std::cout << box.m_name << std::endl;
  //      boxEntityPtr->material()->setAlpha(1.0);
  //  }
  //  else
  //  {
  //      boxEntityPtr->material()->setAlpha(1.0);
  //  }

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
