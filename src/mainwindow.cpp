#include <QDir>
#include "mainwindow.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags f) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  std::cout << "MainWindow constructor" << std::endl;

  MainWindow::singleton_ = this;

  ui->setupUi(this);

  view_ = new Qt3DExtras::Qt3DWindow();

  scene_3d_ = new SceneRenderer3D();

  // camera
  camera_ = view_->camera();
  camera_->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
  camera_->setPosition(QVector3D(0, 0, 10.0f));
  camera_->setViewCenter(QVector3D(0, 0, 0));

  Qt3DCore::QEntity* lightEntity = new Qt3DCore::QEntity(scene_3d_->getScene());
  Qt3DRender::QPointLight* light = new Qt3DRender::QPointLight(lightEntity);
  light->setColor("white");
  light->setIntensity(1);
  lightEntity->addComponent(light);
  Qt3DCore::QTransform* lightTransform = new Qt3DCore::QTransform(lightEntity);
  lightTransform->setTranslation(camera_->position());
  lightEntity->addComponent(lightTransform);

  // manipulator
  manipulator_ = new Qt3DExtras::QFirstPersonCameraController(scene_3d_->getScene());
  manipulator_->setLinearSpeed(50.f);
  manipulator_->setLookSpeed(180.f);
  manipulator_->setCamera(camera_);

  view_->setRootEntity(scene_3d_->getScene());

  QWidget* widget = createWindowContainer(view_, this);
  widget->resize(640, 480);
  ui->verticalLayout->addWidget(widget);

  this->timer_id_ = startTimer(1);
}

MainWindow::~MainWindow()
{
  delete ui;
}

MainWindow* MainWindow::instance()
{
  if (NULL == MainWindow::singleton_)
  {
    new MainWindow();
  }
  return MainWindow::singleton_;
}

void MainWindow::resetCamera()
{
  // camera
  camera_->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
  camera_->setPosition(QVector3D(0, 0, 40.0f));
  camera_->setViewCenter(QVector3D(0, 0, 0));

  manipulator_->setCamera(camera_);
}

void MainWindow::on_resetButton_clicked()
{
  qDebug() << "resetting Camera";
  resetCamera();
}

void MainWindow::on_loadButton_clicked()
{
  std::cout << "Loading boxes" << std::endl;
  QString boxes_file = QCoreApplication::applicationDirPath() + QString("/../../test/data/boxes.json");

  boxes_ = box_factory::BoxJsonParser::getBoxesFromJsonFile(boxes_file);
  std::cout << "Number of boxes " << boxes_.size() << std::endl;

  for (const bpa::Box& b : boxes_)
  {
    ObjectModel model;
    model.object_name = b.m_name.c_str();
    qDebug() << "Name --> " << model.object_name;

    ObjectBody body;
    body.body_name = model.object_name;
    body.pose.position.setX(b.position.position[0]);
    body.pose.position.setY(b.position.position[1]);
    body.pose.position.setZ(b.position.position[2]);

    std::cout << "Position --> " << b.position.position.transpose() << std::endl;
    qDebug() << body.pose.position;
    body.pose.orientation.setX(0);
    body.pose.orientation.setY(0);
    body.pose.orientation.setZ(0);
    body.pose.orientation.setScalar(1);
    body.is_box = true;
    body.bbox.setX(b.m_length);
    body.bbox.setY(b.m_width);
    body.bbox.setZ(b.m_height);

    model.bodies.push_back(body);
    models_.push_back(model);
    scene_3d_->addObjectModel(model);
  }
}

void MainWindow::on_planButton_clicked()
{
  std::cout << "Planning" << std::endl;

  for (bpa::Box& b : boxes_)
  {
    b.position.position[0] -= b.m_length / 2;
    b.position.position[1] -= b.m_width / 2;
    b.position.position[2] -= b.m_height / 2;
  }

  planned_boxes_ = bpp_inf_.binPackingBoxes(boxes_);

  for (bpa::Box& b : planned_boxes_)
  {
    b.position.position[0] += b.m_length / 2;
    b.position.position[1] += b.m_width / 2;
    b.position.position[2] += b.m_height / 2;

    ObjectModel model;
    model.object_name = b.m_name.c_str();
    qDebug() << "Packed Name --> " << model.object_name;

    ObjectBody body;
    body.body_name = model.object_name;
    body.pose.position.setX(b.position.position[0]);
    body.pose.position.setY(b.position.position[1]);
    body.pose.position.setZ(b.position.position[2]);
    std::cout << "Position --> " << b.position.position.transpose() << std::endl;

    double box_rot = (b.is_rotated == true) ? M_PI_2 : 0.0;

    QQuaternion box_orientation = QQuaternion::fromEulerAngles(0, box_rot, 0);

    body.pose.orientation = box_orientation;
    body.is_box = true;
    body.bbox.setX(b.m_length);
    body.bbox.setY(b.m_width);
    body.bbox.setZ(b.m_height);

    model.bodies.push_back(body);
    models_.push_back(model);
    scene_3d_->updateObjectModel(model);
  }
}

void MainWindow::on_deleteButton_clicked()
{
  std::cout << "Deleting model" << std::endl;

  for (ObjectModel model : models_)
  {
    scene_3d_->removeObjectModel(model);
  }

  models_.erase(models_.begin(), models_.end());
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  std::cout << "Closing main gui" << std::endl;
  killTimer(this->timer_id_);

  close();
}

void MainWindow::timerEvent(QTimerEvent* timerEvent)
{
}
